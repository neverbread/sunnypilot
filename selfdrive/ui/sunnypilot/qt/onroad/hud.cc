/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/onroad/hud.h"

#include <algorithm>
#include <cmath>

#include <QPainterPath>

#include "common/timing.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/util.h"

namespace {
constexpr double KPA_TO_PSI = 0.14503773800722;
constexpr double PSI_TO_KPA = 6.8947572932;
// A corner this far below the best inflated of the four reads as low. It is a
// comparison between the car's own tires, not a factory warning threshold.
constexpr double LOW_TIRE_DELTA_PSI = 3.0;
// The strip sits over asphalt, so it needs almost no backing of its own. Kept as
// a named constant because how visible this frame should be is a taste call.
constexpr int TIRE_CARD_ALPHA = 40;
// Estimated stock-range display endpoint, not a verified factory limit.
// N75 describes stock-level boost around 18 psi, occasionally 19 psi:
// https://n75motorsports.ca/products/ecu-tuning-elantra-sport-gtsport-nline
constexpr double BOOST_FULL_RED_PSI = 19.0;
constexpr double VACUUM_FULL_BLUE_KPA = 101.325;  // Sea-level full vacuum, about -14.7 psi gauge.

// The gear badge hangs off the top-right corner of the set-speed card, deliberately the
// smaller of the two so it reads as a footnote to MAX rather than a second speed.
constexpr int GEAR_CARD_WIDTH = 130, GEAR_CARD_HEIGHT = 156, GEAR_CARD_GAP = 16, GEAR_CARD_RADIUS = 32;
constexpr int GEAR_CAPTION_PX = 34, GEAR_VALUE_PX = 72;
constexpr int GEAR_CAPTION_OFFSET = 20, GEAR_VALUE_OFFSET = 58;
// engineTemperature publishes the gear as its raw value: 0 means no gear is engaged and 14 is
// the invalid sentinel, so anything outside a real seven-speed box is treated as no reading.
constexpr int GEAR_MIN = 1, GEAR_MAX = 7;
// The split camera view halves the screen, and a 176 px speed numeral centred on that half
// reaches back to within about 15 px of this card's right border, so the pair needs the wider
// guard the tire strip uses rather than the one the other readings accept.
constexpr int GEAR_MIN_SURFACE_WIDTH = 1100;

// engineTemperature publishes the clutch states as plain numbers, not an enum.
constexpr int CLUTCH_UNKNOWN = 0, CLUTCH_DISENGAGED = 1, CLUTCH_ENGAGED = 2, CLUTCH_SLIPPING = 3;
// The reader already holds the last decoded pair for five seconds. Holding it
// again here keeps a single dropped publish from flashing the glyphs off.
constexpr uint64_t CLUTCH_HOLD_NANOS = 10000000000ULL;
}  // namespace

HudRendererSP::HudRendererSP() {}

void HudRendererSP::updateState(const UIState &s) {
  HudRenderer::updateState(s);

  const SubMaster &sm = *s.sm;
  const uint64_t now = nanos_since_boot();
  const auto fresh_service = [&](const char *service) {
    return sm.alive(service) && sm.valid(service) && sm.rcv_frame(service) > s.scene.started_frame &&
           now - sm.rcv_time(service) <= 500000000ULL;
  };
  // Keep brake status independent of the optional vehicle readings below.
  // The pedal remains available on cars without a reported brake-light signal.
  const bool braking = s.scene.started && s.scene.braking_status && fresh_service("carState") &&
                       (sm["carState"].getCarState().getBrakePressed() ||
                        (fresh_service("carStateSP") && sm["carStateSP"].getCarStateSP().getBrakeLights()));
  current_speed_color = braking ? QColor(255, 0, 0) : QColor(Qt::white);

  // Hide supplementary readings during alerts and while selfdrive is unavailable.
  const bool selfdrive_ready = sm.alive("selfdriveState") && sm.valid("selfdriveState") &&
                              sm.rcv_frame("selfdriveState") > s.scene.started_frame &&
                              now - sm.rcv_time("selfdriveState") <= 3e9;
  const bool alert_visible = selfdrive_ready &&
                            sm["selfdriveState"].getSelfdriveState().getAlertSize() != cereal::SelfdriveState::AlertSize::NONE;
  const bool readings_visible = s.scene.started && selfdrive_ready && !alert_visible;

  // The tire pressures ride on carStateSP, not on the optional diagnostic
  // service below, so they keep working when every temperature toggle is off.
  show_tire_pressures = false;
  tpms_warning = false;
  if (readings_visible && fresh_service("carStateSP")) {
    const auto tire_state = sm["carStateSP"].getCarStateSP();
    const auto reported_pressures = tire_state.getTirePressuresPsi();
    if (tire_state.getTirePressureValid() && reported_pressures.size() == 4) {
      for (int i = 0; i < 4; ++i) tire_pressures_psi[i] = reported_pressures[i];
      tpms_warning = tire_state.getTpmsWarning();
      show_tire_pressures = s.scene.tire_pressure_display;
    }
  }

  const bool current_drive = sm.rcv_frame("engineTemperature") > s.scene.started_frame;
  show_engine_temperatures = false;
  show_dct_temperatures = false;
  show_boost = false;
  show_clutch_states = false;
  show_gear = false;
  gear_text.clear();
  oil_background = Qt::transparent;
  boost_background = Qt::transparent;
  if (!current_drive) return;
  const auto temperatures = sm["engineTemperature"].getEngineTemperature();
  const bool visible = current_drive && temperatures.getSupported() && readings_visible;
  show_engine_temperatures = s.scene.engine_temperature_display && visible;
  show_dct_temperatures = s.scene.dct_temperature_display && visible;
  show_boost = s.scene.boost_display && visible;
  show_clutch_states = s.scene.clutch_state_display && visible;
  const bool fresh = current_drive && sm.alive("engineTemperature") && sm.valid("engineTemperature") &&
                     now - sm.rcv_time("engineTemperature") <= 3e9;
  const QString unit = is_metric ? QStringLiteral("\u00b0C") : QStringLiteral("\u00b0F");
  auto temperature_text = [&](float celsius, bool valid) {
    const QString value = fresh && valid && std::isfinite(celsius)
                            ? QString::number(std::round(is_metric ? celsius : celsius * 1.8 + 32.0), 'f', 0)
                            : QStringLiteral("\u2014");
    return value + unit;
  };
  const float oil_celsius = temperatures.getOilCelsius();
  oil_temperature = temperature_text(oil_celsius, temperatures.getOilValid());
  if (fresh && temperatures.getOilValid() && std::isfinite(oil_celsius)) {
    // 140 F = 60 C; 176 F = 80 C. Use the unrounded reading in either display unit.
    // A light tint over the dark card preserves transparency and bright text.
    oil_background = oil_celsius <= 60.0f ? QColor(255, 145, 145, 50) :
                     oil_celsius <= 80.0f ? QColor(255, 221, 128, 50) : QColor(144, 238, 168, 50);
  }
  coolant_temperature = temperature_text(temperatures.getCoolantCelsius(), temperatures.getCoolantValid());
  clutch1_temperature = temperature_text(temperatures.getClutch1Celsius(), temperatures.getClutch1Valid());
  clutch2_temperature = temperature_text(temperatures.getClutch2Celsius(), temperatures.getClutch2Valid());
  const float boost_kpa = temperatures.getBoostKpa();
  const bool boost_valid = fresh && temperatures.getBoostValid() && std::isfinite(boost_kpa);
  const double boost_psi = boost_kpa * KPA_TO_PSI;
  const QString boost_value = boost_valid
                                ? QString::number(is_metric ? boost_kpa : boost_psi, 'f', 1)
                                : QStringLiteral("\u2014");
  boost_pressure = boost_value + (is_metric ? QStringLiteral(" kPa") : QStringLiteral(" psi"));
  if (boost_valid) {
    const double fraction = std::clamp(boost_kpa < 0.0f ? -boost_kpa / VACUUM_FULL_BLUE_KPA : boost_psi / BOOST_FULL_RED_PSI, 0.0, 1.0);
    // Emphasize modest pressures for peripheral visibility, fading to neutral
    // at zero. Cap only the tint; the numeric reading remains unrestricted.
    const int alpha = static_cast<int>(std::lround(190.0 * std::sqrt(fraction)));
    boost_background = boost_kpa < 0.0f ? QColor(35, 105, 245, alpha) : QColor(230, 45, 55, alpha);
  }
  if (fresh) {
    // An unknown pair is a real reading — neutral, or the transmission not
    // reporting a gear — so take it as published while the service is fresh.
    clutch1_state = static_cast<int>(temperatures.getClutch1State());
    clutch2_state = static_cast<int>(temperatures.getClutch2State());
    last_clutch_state_nanos = now;
  } else if (now - last_clutch_state_nanos > CLUTCH_HOLD_NANOS) {
    clutch1_state = clutch2_state = CLUTCH_UNKNOWN;
  }
  // The gear rides in the same TCM reply as the clutch readings, and the GearDisplay toggle keeps
  // enginetempd polling that reply on its own, so the badge does not depend on the DCT toggles.
  // It is not hidden by an alert the way the diagnostic strip is: it belongs to the set-speed card
  // it is attached to, and that card stays up.
  if (fresh && s.scene.gear_display) {
    const float reported_gear = temperatures.getGear();
    if (std::isfinite(reported_gear)) {
      const int gear = static_cast<int>(std::lround(reported_gear));
      if (GEAR_MIN <= gear && gear <= GEAR_MAX) {
        gear_text = QString::number(gear);
        show_gear = true;
      }
    }
  }
}

void HudRendererSP::draw(QPainter &p, const QRect &surface_rect) {
  HudRenderer::draw(p, surface_rect);
  drawGearIndicator(p, surface_rect);
  drawTirePressures(p, surface_rect);
  drawVehicleReadings(p, surface_rect);
  drawClutchIndicators(p, surface_rect);
}

void HudRendererSP::drawGearIndicator(QPainter &p, const QRect &surface_rect) {
  // Attached to the set-speed card, so with that card absent there is nothing to attach to.
  if (!show_gear || !is_cruise_available || surface_rect.width() < GEAR_MIN_SURFACE_WIDTH) return;
  const QRect max_card = setSpeedRect();
  const QRect card(max_card.right() + 1 + GEAR_CARD_GAP, max_card.top(), GEAR_CARD_WIDTH, GEAR_CARD_HEIGHT);

  // Same frame and backing as the set-speed card, so the pair reads as one instrument.
  p.setPen(QPen(QColor(255, 255, 255, 75), 6));
  p.setBrush(QColor(0, 0, 0, 166));
  p.drawRoundedRect(card, GEAR_CARD_RADIUS, GEAR_CARD_RADIUS);

  p.setFont(InterFont(GEAR_CAPTION_PX, QFont::DemiBold));
  p.setPen(QColor(0xa6, 0xa6, 0xa6, 0xff));
  p.drawText(card.adjusted(0, GEAR_CAPTION_OFFSET, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("GEAR"));

  p.setFont(InterFont(GEAR_VALUE_PX, QFont::Bold));
  p.setPen(Qt::white);
  p.drawText(card.adjusted(0, GEAR_VALUE_OFFSET, 0, 0), Qt::AlignTop | Qt::AlignHCenter, gear_text);
}

void HudRendererSP::drawVehicleReadings(QPainter &p, const QRect &surface_rect) {
  const int row_count = 2 * (int(show_engine_temperatures) + int(show_dct_temperatures)) + int(show_boost);
  if (row_count == 0 || surface_rect.width() < 800) return;

  // Left column. Use a wider card for readable values, leaving room for the speed
  // display in split camera views. Compact five rows to stay between the buttons.
  const bool compact = row_count == 5;
  const int card_width = std::min(320, surface_rect.width() / 2 - 150);
  const int row_height = compact ? 106 : 128;
  const QRect card(surface_rect.left() + UI_BORDER_SIZE,
                   surface_rect.top() + UI_BORDER_SIZE + btn_size + 20, card_width, row_count * row_height + 4);
  // The driver monitoring icon sits in this column's lower corner, so a card that
  // would reach it is dropped rather than overlapped.
  if (card.bottom() + 20 >= surface_rect.bottom() + 1 - UI_BORDER_SIZE - btn_size) return;

  p.save();
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 185));
  p.drawRoundedRect(card, 24, 24);
  int row = 0;
  auto draw_reading = [&](const QString &label, const QString &value, const QColor &background = QColor(Qt::transparent)) {
    if (background.alpha() > 0) {
      p.save();
      QPainterPath card_clip;
      card_clip.addRoundedRect(card, 24, 24);
      p.setClipPath(card_clip, Qt::IntersectClip);
      const int row_top = card.y() + row * row_height;
      const int row_bottom = row == row_count - 1 ? card.bottom() + 1 : row_top + row_height;
      p.fillRect(QRect(card.x(), row_top, card.width(), row_bottom - row_top), background);
      p.restore();
    }
    const int top_padding = compact ? 4 : 8;
    const int top = card.y() + top_padding + row * row_height;
    if (row > 0) {
      p.setPen(QPen(QColor(255, 255, 255, 35), 1));
      p.drawLine(card.x() + 24, top - top_padding, card.right() - 24, top - top_padding);
    }
    const QRect label_rect(card.x() + 20, top, card.width() - 40, compact ? 40 : 42);
    p.setFont(InterFont(compact ? 32 : 34, QFont::DemiBold));
    p.setPen(QColor(255, 255, 255, 225));
    p.drawText(label_rect, Qt::AlignCenter, p.fontMetrics().elidedText(label, Qt::ElideRight, label_rect.width()));
    int value_font_size = compact ? 52 : 60;
    p.setFont(InterFont(value_font_size, QFont::Bold));
    while (p.fontMetrics().horizontalAdvance(value) > label_rect.width() && value_font_size > 24) {
      p.setFont(InterFont(--value_font_size, QFont::Bold));
    }
    p.setPen(Qt::white);
    p.drawText(QRect(card.x() + 20, top + (compact ? 36 : 40), card.width() - 40, compact ? 64 : 76), Qt::AlignCenter,
               p.fontMetrics().elidedText(value, Qt::ElideRight, label_rect.width()));
    ++row;
  };
  if (show_engine_temperatures) {
    draw_reading(tr("OIL"), oil_temperature, oil_background);
    draw_reading(tr("COOLANT"), coolant_temperature);
  }
  if (show_dct_temperatures) {
    draw_reading(tr("CLUTCH 1"), clutch1_temperature);
    draw_reading(tr("CLUTCH 2"), clutch2_temperature);
  }
  if (show_boost) {
    draw_reading(tr("BOOST"), boost_pressure, boost_background);
  }
  p.restore();
}

void HudRendererSP::drawClutchIndicators(QPainter &p, const QRect &surface_rect) {
  if (!show_clutch_states || surface_rect.width() < 800) return;

  // Right column, centred between the same anchors the readings card uses, which
  // keeps the glyphs below the set-speed card. The driver monitoring icon lives in
  // the left column's lower corner, so this side has the height to spare.
  // Stacked rather than side by side: the column has height to spare and the
  // glyphs can be much larger without reaching toward the speed numeral.
  const int glyph_size = 104, glyph_gap = 22, panel_padding = 20, caption_height = 40;
  const int panel_height = caption_height + 2 * glyph_size + glyph_gap + 2 * panel_padding;
  const int top_anchor = surface_rect.top() + UI_BORDER_SIZE + btn_size + 20;
  const int bottom_anchor = surface_rect.bottom() + 1 - UI_BORDER_SIZE - btn_size;
  if (panel_height > bottom_anchor - top_anchor) return;  // Short surfaces cannot fit the stack.
  const int panel_width = glyph_size + 2 * panel_padding;
  const QRect panel(surface_rect.right() + 1 - UI_BORDER_SIZE - panel_width,
                    (top_anchor + bottom_anchor - panel_height) / 2, panel_width, panel_height);

  p.save();
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 185));
  p.drawRoundedRect(panel, 24, 24);
  p.setFont(InterFont(26, QFont::DemiBold));
  p.setPen(QColor(255, 255, 255, 180));
  p.drawText(QRect(panel.x() + panel_padding, panel.y() + panel_padding - 4,
                   panel.width() - 2 * panel_padding, caption_height), Qt::AlignCenter, tr("CLUTCH"));

  const QColor amber(255, 178, 32);
  const int states[2] = {clutch1_state, clutch2_state};
  for (int i = 0; i < 2; ++i) {
    const QRectF circle(panel.x() + panel_padding,
                        panel.y() + panel_padding + caption_height + i * (glyph_size + glyph_gap),
                        glyph_size, glyph_size);
    if (states[i] == CLUTCH_SLIPPING) {
      // Half fill reads as a partly engaged clutch. Clip the fill alone so the
      // amber ring keeps its full circumference.
      p.save();
      QPainterPath lower_half;
      lower_half.addRect(QRectF(circle.left(), circle.center().y(), circle.width(), circle.height() / 2.0));
      p.setClipPath(lower_half, Qt::IntersectClip);
      p.setPen(Qt::NoPen);
      p.setBrush(amber);
      p.drawEllipse(circle);
      p.restore();
    }

    QColor fill = Qt::transparent, text = QColor(255, 255, 255, 150);
    QPen pen(QColor(255, 255, 255, 110), 5);
    switch (states[i]) {
      case CLUTCH_ENGAGED:
        fill = QColor(255, 255, 255, 235);
        pen = QPen(QColor(255, 255, 255, 60), 3);
        text = QColor(20, 20, 20, 235);
        break;
      case CLUTCH_SLIPPING:
        pen = QPen(amber, 6);
        text = QColor(255, 255, 255, 240);
        break;
      case CLUTCH_DISENGAGED:
        break;
      default:
        pen = QPen(QColor(255, 255, 255, 60), 4, Qt::CustomDashLine);
        pen.setDashPattern({7, 7});
        text = QColor(255, 255, 255, 70);
        break;
    }
    p.setBrush(fill);
    p.setPen(pen);
    p.drawEllipse(circle);
    p.setFont(InterFont(46, QFont::Bold));
    p.setPen(text);
    p.drawText(circle, Qt::AlignCenter, QString::number(i + 1));
  }
  p.restore();
}

void HudRendererSP::drawTirePressures(QPainter &p, const QRect &surface_rect) {
  if (!show_tire_pressures || surface_rect.width() < 1100) return;

  // Bottom edge of the camera surface, level with the driver monitoring box on
  // the left. Alerts use this same band, but updateState already hides the
  // strip while an alert is up, so the two never compete for the space.
  const int cell_width = 148, unit_width = 60, card_height = 128, card_padding = 18;
  const int card_width = 4 * cell_width + unit_width + 2 * card_padding;
  const QRect card((surface_rect.left() + surface_rect.right() + 1 - card_width) / 2,
                   surface_rect.bottom() + 1 - card_height, card_width, card_height);
  if (card.left() < surface_rect.left() + UI_BORDER_SIZE) return;  // Split views leave no room.

  float highest_psi = 0.0f;
  for (const float pressure : tire_pressures_psi) highest_psi = std::max(highest_psi, pressure);

  p.save();
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, TIRE_CARD_ALPHA));
  p.drawRoundedRect(card, 24, 24);

  const QColor normal(255, 255, 255, 235);
  const QColor low(255, 178, 32);  // The same amber the clutch glyphs use.
  const char *labels[] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; ++i) {
    const QRect cell(card.x() + card_padding + i * cell_width, card.y(), cell_width, card_height);
    // The vehicle's own telltale wins: it knows a threshold this comparison cannot.
    const bool low_corner = tpms_warning || highest_psi - tire_pressures_psi[i] >= LOW_TIRE_DELTA_PSI;
    p.setFont(InterFont(30, QFont::DemiBold));
    p.setPen(QColor(255, 255, 255, 150));
    p.drawText(QRect(cell.x(), cell.y() + 12, cell.width(), 34), Qt::AlignCenter, QString::fromLatin1(labels[i]));
    const QString value = QString::number(is_metric ? tire_pressures_psi[i] * PSI_TO_KPA : tire_pressures_psi[i], 'f', 0);
    p.setFont(InterFont(56, QFont::Bold));
    p.setPen(low_corner ? low : normal);
    p.drawText(QRect(cell.x(), cell.y() + 46, cell.width(), 66), Qt::AlignCenter, value);
  }

  p.setFont(InterFont(30, QFont::DemiBold));
  p.setPen(QColor(255, 255, 255, 150));
  p.drawText(QRect(card.right() - card_padding - unit_width, card.y() + (card_height - 34) / 2, unit_width, 34),
             Qt::AlignCenter, is_metric ? QStringLiteral("kPa") : QStringLiteral("psi"));
  p.restore();
}

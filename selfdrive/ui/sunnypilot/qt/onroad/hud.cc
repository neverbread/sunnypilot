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
// Estimated stock-range display endpoint, not a verified factory limit.
// N75 describes stock-level boost around 18 psi, occasionally 19 psi:
// https://n75motorsports.ca/products/ecu-tuning-elantra-sport-gtsport-nline
constexpr double BOOST_FULL_RED_PSI = 19.0;
constexpr double VACUUM_FULL_BLUE_KPA = 101.325;  // Sea-level full vacuum, about -14.7 psi gauge.
}  // namespace

HudRendererSP::HudRendererSP() {}

void HudRendererSP::updateState(const UIState &s) {
  HudRenderer::updateState(s);

  const SubMaster &sm = *s.sm;
  const uint64_t now = nanos_since_boot();
  const auto fresh_brake_state = [&](const char *service) {
    return sm.alive(service) && sm.valid(service) && sm.rcv_frame(service) > s.scene.started_frame &&
           now - sm.rcv_time(service) <= 500000000ULL;
  };
  // Keep brake status independent of the optional vehicle readings below.
  // The pedal remains available on cars without a reported brake-light signal.
  const bool braking = s.scene.started && s.scene.braking_status && fresh_brake_state("carState") &&
                       (sm["carState"].getCarState().getBrakePressed() ||
                        (fresh_brake_state("carStateSP") && sm["carStateSP"].getCarStateSP().getBrakeLights()));
  current_speed_color = braking ? QColor(255, 0, 0) : QColor(Qt::white);

  const bool current_drive = sm.rcv_frame("engineTemperature") > s.scene.started_frame;
  show_engine_temperatures = false;
  show_dct_temperatures = false;
  show_boost = false;
  oil_background = Qt::transparent;
  boost_background = Qt::transparent;
  if (!current_drive) return;
  const auto temperatures = sm["engineTemperature"].getEngineTemperature();
  // Hide supplementary readings during alerts and while selfdrive is unavailable.
  const bool selfdrive_ready = sm.alive("selfdriveState") && sm.valid("selfdriveState") &&
                              sm.rcv_frame("selfdriveState") > s.scene.started_frame &&
                              now - sm.rcv_time("selfdriveState") <= 3e9;
  const bool alert_visible = selfdrive_ready &&
                            sm["selfdriveState"].getSelfdriveState().getAlertSize() != cereal::SelfdriveState::AlertSize::NONE;
  const bool visible = s.scene.started && current_drive && temperatures.getSupported() && selfdrive_ready && !alert_visible;
  show_engine_temperatures = s.scene.engine_temperature_display && visible;
  show_dct_temperatures = s.scene.dct_temperature_display && visible;
  show_boost = s.scene.boost_display && visible;
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
}

void HudRendererSP::draw(QPainter &p, const QRect &surface_rect) {
  HudRenderer::draw(p, surface_rect);
  drawVehicleReadings(p, surface_rect);
}

void HudRendererSP::drawVehicleReadings(QPainter &p, const QRect &surface_rect) {
  const int row_count = 2 * (int(show_engine_temperatures) + int(show_dct_temperatures)) + int(show_boost);
  if (row_count == 0 || surface_rect.width() < 800) return;

  // Use a wider card for readable values, leaving room for the speed display
  // in split camera views. Compact five rows to stay between the buttons.
  const bool compact = row_count == 5;
  const int card_width = std::min(320, surface_rect.width() / 2 - 150);
  const int row_height = compact ? 106 : 128;
  const QRect card(surface_rect.right() + 1 - UI_BORDER_SIZE - card_width,
                   surface_rect.top() + UI_BORDER_SIZE + btn_size + 20, card_width, row_count * row_height + 4);
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

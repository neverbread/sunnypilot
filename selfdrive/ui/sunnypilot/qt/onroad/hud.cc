/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/onroad/hud.h"

#include <cmath>

#include "common/timing.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/util.h"

HudRendererSP::HudRendererSP() {}

void HudRendererSP::updateState(const UIState &s) {
  HudRenderer::updateState(s);

  const SubMaster &sm = *s.sm;
  const uint64_t now = nanos_since_boot();
  const bool current_drive = sm.rcv_frame("engineTemperature") > s.scene.started_frame;
  show_engine_temperatures = false;
  show_dct_temperatures = false;
  show_boost = false;
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
  oil_temperature = temperature_text(temperatures.getOilCelsius(), temperatures.getOilValid());
  coolant_temperature = temperature_text(temperatures.getCoolantCelsius(), temperatures.getCoolantValid());
  clutch1_temperature = temperature_text(temperatures.getClutch1Celsius(), temperatures.getClutch1Valid());
  clutch2_temperature = temperature_text(temperatures.getClutch2Celsius(), temperatures.getClutch2Valid());
  const float boost_kpa = temperatures.getBoostKpa();
  const QString boost_value = fresh && temperatures.getBoostValid() && std::isfinite(boost_kpa)
                                ? QString::number(is_metric ? boost_kpa : boost_kpa * 0.14503773800722, 'f', 1)
                                : QStringLiteral("\u2014");
  boost_pressure = boost_value + (is_metric ? QStringLiteral(" kPa") : QStringLiteral(" psi"));
}

void HudRendererSP::draw(QPainter &p, const QRect &surface_rect) {
  HudRenderer::draw(p, surface_rect);
  drawVehicleReadings(p, surface_rect);
}

void HudRendererSP::drawVehicleReadings(QPainter &p, const QRect &surface_rect) {
  const int row_count = 2 * (int(show_engine_temperatures) + int(show_dct_temperatures)) + int(show_boost);
  if (row_count == 0 || surface_rect.width() < 800) return;

  // A narrow card fits beside the speed display even in a split camera view.
  // Compact five rows to keep the card clear of both upper and lower buttons.
  const bool compact = row_count == 5;
  const int row_height = compact ? 96 : 107;
  const QRect card(surface_rect.right() + 1 - UI_BORDER_SIZE - 236,
                   surface_rect.top() + UI_BORDER_SIZE + btn_size + 20, 236, row_count * row_height + 4);
  if (card.bottom() + 20 >= surface_rect.bottom() + 1 - UI_BORDER_SIZE - btn_size) return;

  p.save();
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(card, 24, 24);
  int row = 0;
  auto draw_reading = [&](const QString &label, const QString &value) {
    const int top_padding = compact ? 8 : 12;
    const int top = card.y() + top_padding + row * row_height;
    if (row > 0) {
      p.setPen(QPen(QColor(255, 255, 255, 35), 1));
      p.drawLine(card.x() + 24, top - top_padding, card.right() - 24, top - top_padding);
    }
    const QRect label_rect(card.x() + 16, top, card.width() - 32, compact ? 30 : 34);
    p.setFont(InterFont(compact ? 26 : 28, QFont::DemiBold));
    p.setPen(QColor(255, 255, 255, 175));
    p.drawText(label_rect, Qt::AlignCenter, p.fontMetrics().elidedText(label, Qt::ElideRight, label_rect.width()));
    int value_font_size = compact ? 42 : 44;
    p.setFont(InterFont(value_font_size, QFont::DemiBold));
    while (p.fontMetrics().horizontalAdvance(value) > label_rect.width() && value_font_size > 24) {
      p.setFont(InterFont(--value_font_size, QFont::DemiBold));
    }
    p.setPen(QColor(255, 255, 255, 235));
    p.drawText(QRect(card.x() + 16, top + (compact ? 29 : 32), card.width() - 32, compact ? 52 : 54), Qt::AlignCenter,
               p.fontMetrics().elidedText(value, Qt::ElideRight, label_rect.width()));
    ++row;
  };
  if (show_engine_temperatures) {
    draw_reading(tr("OIL"), oil_temperature);
    draw_reading(tr("COOLANT"), coolant_temperature);
  }
  if (show_dct_temperatures) {
    draw_reading(tr("CLUTCH 1"), clutch1_temperature);
    draw_reading(tr("CLUTCH 2"), clutch2_temperature);
  }
  if (show_boost) {
    draw_reading(tr("BOOST"), boost_pressure);
  }
  p.restore();
}

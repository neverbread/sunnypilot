/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include <cstdint>

#include <QColor>
#include <QPainter>

#include "selfdrive/ui/qt/onroad/hud.h"

class HudRendererSP : public HudRenderer {
  Q_OBJECT

public:
  HudRendererSP();
  void updateState(const UIState &s) override;
  void draw(QPainter &p, const QRect &surface_rect) override;

private:
  void drawVehicleReadings(QPainter &p, const QRect &surface_rect);
  void drawGearIndicator(QPainter &p, const QRect &surface_rect);
  void drawClutchIndicators(QPainter &p, const QRect &surface_rect);
  void drawTirePressures(QPainter &p, const QRect &surface_rect);
  bool show_engine_temperatures = false;
  bool show_dct_temperatures = false;
  bool show_boost = false;
  bool show_clutch_states = false;
  bool show_tire_pressures = false;
  bool show_gear = false;
  // Front left, front right, rear left, rear right, as the vehicle reports them.
  float tire_pressures_psi[4] = {};
  bool tpms_warning = false;
  // 0 unknown / 1 disengaged / 2 engaged / 3 slipping, as engineTemperature publishes them.
  int clutch1_state = 0, clutch2_state = 0;
  uint64_t last_clutch_state_nanos = 0;
  QColor oil_background = Qt::transparent;
  QColor boost_background = Qt::transparent;
  QString oil_temperature;
  QString coolant_temperature;
  QString clutch1_temperature;
  QString clutch2_temperature;
  QString boost_pressure;
  QString gear_text;
};

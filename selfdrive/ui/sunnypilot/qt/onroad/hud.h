/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

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
  bool show_engine_temperatures = false;
  bool show_dct_temperatures = false;
  bool show_boost = false;
  QString oil_temperature;
  QString coolant_temperature;
  QString clutch1_temperature;
  QString clutch2_temperature;
  QString boost_pressure;
};

/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include <optional>

#include "selfdrive/ui/qt/onroad/model.h"

class ModelRendererSP : public ModelRenderer {
public:
  ModelRendererSP() = default;

private:
  void updatePathState(const UIState &s);
  void update_model(const cereal::ModelDataV2::Reader &model, const cereal::RadarState::LeadData::Reader &lead) override;
  void drawPath(QPainter &painter, const cereal::ModelDataV2::Reader &model, const QRect &rect) override;
  void mapLaneToPolygon(const cereal::XYZTData::Reader &left_line, const cereal::XYZTData::Reader &right_line,
                        QPolygonF *vertices, int max_idx);

  QPolygonF left_blindspot_vertices;
  QPolygonF right_blindspot_vertices;
  QPolygonF blindspot_lane_vertices[2];
  bool lateral_active = false;
  std::optional<uint64_t> path_started_frame;
  FirstOrderFilter path_width_filter{0.4f, 0.1f, 1.0f / UI_FREQ};
};

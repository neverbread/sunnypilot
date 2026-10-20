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
  void drawLeadTrackingBox(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_one,
                           const cereal::XYZTData::Reader &position, const QRect &surface_rect) override;
  void mapLaneToPolygon(const cereal::XYZTData::Reader &left_line, const cereal::XYZTData::Reader &right_line,
                        QPolygonF *vertices, int max_idx);

  QPolygonF left_blindspot_vertices;
  QPolygonF right_blindspot_vertices;
  QPolygonF blindspot_lane_vertices[2];
  bool lateral_active = false;
  std::optional<uint64_t> path_started_frame;
  FirstOrderFilter path_width_filter{0.4f, 0.1f, 1.0f / UI_FREQ};
  // When the departure reminder last confirmed its lead pulling away, so the box can hold its
  // green frame for a moment. Self-expiring, so it carries nothing between stops.
  uint64_t lead_depart_nanos = 0;
};

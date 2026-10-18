/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#include "selfdrive/ui/sunnypilot/qt/onroad/model.h"

#include <algorithm>
#include <cmath>

#include "common/timing.h"

void ModelRendererSP::updatePathState(const UIState &s) {
  const auto &sm = *s.sm;
  const uint64_t now = nanos_since_boot();
  const auto fresh = [&](const char *service) {
    return sm.alive(service) && sm.valid(service) && sm.rcv_frame(service) > s.scene.started_frame &&
           now - sm.rcv_time(service) <= 3e9;
  };
  const auto mads = sm["selfdriveStateSP"].getSelfdriveStateSP().getMads();
  lateral_active = false;
  if (s.scene.started) {
    if (mads.getAvailable()) {
      // Overrides retain the wide path while MADS is enabled; a pause does not.
      lateral_active = fresh("selfdriveStateSP") && mads.getEnabled() &&
                       mads.getState() != cereal::ModularAssistiveDrivingSystem::ModularAssistiveDrivingSystemState::PAUSED;
    } else {
      lateral_active = fresh("selfdriveState") && (s.status == STATUS_ENGAGED || s.status == STATUS_LAT_ONLY);
    }
  }

  const float target_width = lateral_active ? 0.9f : 0.4f;
  if (path_started_frame != s.scene.started_frame) {
    // Do not carry a previous drive's engagement appearance into this drive.
    path_started_frame = s.scene.started_frame;
    path_width_filter.reset(target_width);
    prev_path_color_enabled = lateral_active;
    blend_factor = 1.0f;
  }
  path_half_width = path_width_filter.update(target_width);
}

void ModelRendererSP::mapLaneToPolygon(const cereal::XYZTData::Reader &left_line, const cereal::XYZTData::Reader &right_line,
                                      QPolygonF *vertices, int max_idx) {
  const auto lx = left_line.getX(), ly = left_line.getY(), lz = left_line.getZ();
  const auto rx = right_line.getX(), ry = right_line.getY(), rz = right_line.getZ();
  const int count = std::min({max_idx + 1, int(lx.size()), int(ly.size()), int(lz.size()),
                              int(rx.size()), int(ry.size()), int(rz.size())});
  QPolygonF left_points, right_points;
  auto project = [&](float x, float y, float z, QPointF *point) {
    const Eigen::Vector3f projected = car_space_transform * Eigen::Vector3f(x, y, z);
    if (!projected.allFinite() || projected.z() <= 1e-3f) return false;
    *point = QPointF(projected.x() / projected.z(), projected.y() / projected.z());
    return std::isfinite(point->x()) && std::isfinite(point->y());
  };
  for (int i = 0; i < count; ++i) {
    if (lx[i] < 0 || rx[i] < 0) continue;
    QPointF left, right;
    if (project(lx[i], ly[i], lz[i], &left) && project(rx[i], ry[i], rz[i], &right)) {
      // Avoid folding the lane region back over itself at a hill crest.
      if (!left_points.isEmpty() && (left.y() > left_points.back().y() || right.y() > right_points.front().y())) continue;
      left_points.push_back(left);
      right_points.push_front(right);
    }
  }
  // Clip after building the full region so an offscreen boundary does not hide the visible lane.
  const QPolygonF lane = left_points + right_points;
  *vertices = lane.intersected(QPolygonF(clip_region));
}

void ModelRendererSP::update_model(const cereal::ModelDataV2::Reader &model, const cereal::RadarState::LeadData::Reader &lead) {
  updatePathState(*uiState());
  ModelRenderer::update_model(model, lead);
  const auto &model_position = model.getPosition();
  const auto &lane_lines = model.getLaneLines();
  float max_distance = std::clamp(*(model_position.getX().end() - 1), MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  // update blindspot vertices
  float max_distance_barrier = 100;
  int max_idx_barrier = std::min(max_idx, get_path_length_idx(lane_lines[0], max_distance_barrier));
  mapLineToPolygon(model.getLaneLines()[1], 0.2, -0.05, &left_blindspot_vertices, max_idx_barrier);
  mapLineToPolygon(model.getLaneLines()[2], 0.2, -0.05, &right_blindspot_vertices, max_idx_barrier);
  mapLaneToPolygon(lane_lines[0], lane_lines[1], &blindspot_lane_vertices[0], max_idx_barrier);
  mapLaneToPolygon(lane_lines[2], lane_lines[3], &blindspot_lane_vertices[1], max_idx_barrier);
}

void ModelRendererSP::drawPath(QPainter &painter, const cereal::ModelDataV2::Reader &model, const QRect &surface_rect) {
  auto *s = uiState();
  auto &sm = *(s->sm);
  bool blindspot = Params().getBool("BlindSpot");

  const bool blindspot_data_fresh = sm.alive("carState") && sm.valid("carState") && sm.alive("modelV2") && sm.valid("modelV2") &&
                                   sm.rcv_frame("carState") > s->scene.started_frame && sm.rcv_frame("modelV2") > s->scene.started_frame;
  if (s->scene.blindspot_lane_highlight && blindspot_data_fresh) {
    const auto car_state = sm["carState"].getCarState();
    painter.save();
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(201, 34, 49, 105));
    if (car_state.getLeftBlindspot()) painter.drawPolygon(blindspot_lane_vertices[0]);
    if (car_state.getRightBlindspot()) painter.drawPolygon(blindspot_lane_vertices[1]);
    painter.restore();
  } else if (!s->scene.blindspot_lane_highlight && blindspot && blindspot_data_fresh) {
    bool left_blindspot = sm["carState"].getCarState().getLeftBlindspot();
    bool right_blindspot = sm["carState"].getCarState().getRightBlindspot();

    //painter.setBrush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.4));  // Red with alpha for blind spot

    if (left_blindspot && !left_blindspot_vertices.isEmpty()) {
      QLinearGradient gradient(0, 0, surface_rect.width(), 0); // Horizontal gradient from left to right
      gradient.setColorAt(0.0, QColor(255, 165, 0, 102)); // Orange with alpha
      gradient.setColorAt(1.0, QColor(255, 255, 0, 102)); // Yellow with alpha
      painter.setBrush(gradient);
      painter.drawPolygon(left_blindspot_vertices);
    }

    if (right_blindspot && !right_blindspot_vertices.isEmpty()) {
      QLinearGradient gradient(surface_rect.width(), 0, 0, 0); // Horizontal gradient from right to left
      gradient.setColorAt(0.0, QColor(255, 165, 0, 102)); // Orange with alpha
      gradient.setColorAt(1.0, QColor(255, 255, 0, 102)); // Yellow with alpha
      painter.setBrush(gradient);
      painter.drawPolygon(right_blindspot_vertices);
    }
  }

  bool rainbow = Params().getBool("RainbowMode");
  //float v_ego = sm["carState"].getCarState().getVEgo();

  if (rainbow && lateral_active) {
    // Simple time-based animation
    float time_offset = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() / 1000.0f;

    // simple linear gradient from bottom to top
    QLinearGradient bg(0, surface_rect.height(), 0, 0);

    // evenly spaced colors across the spectrum
    // The animation shifts the entire spectrum smoothly
    float animation_speed = 40.0f; // speed vroom vroom
    float hue_offset = fmod(time_offset * animation_speed, 360.0f);

    // 6-8 color stops for smooth transitions more color makes it laggy
    const int num_stops = 7;
    for (int i = 0; i < num_stops; i++) {
      float position = static_cast<float>(i) / (num_stops - 1);

      float hue = fmod(hue_offset + position * 360.0f, 360.0f);
      float saturation = 0.9f;
      float lightness = 0.6f;

      // Alpha fades out towards the far end of the path
      float alpha = 0.8f * (1.0f - position * 0.3f);

      QColor color = QColor::fromHslF(hue / 360.0f, saturation, lightness, alpha);
      bg.setColorAt(position, color);
    }

    painter.setBrush(bg);
    painter.drawPolygon(track_vertices);
  } else if (experimental_mode && lateral_active) {
    ModelRenderer::drawPath(painter, model, surface_rect.height());
  } else {
    // Lateral engagement controls the normal path color independently of ACC.
    // Inactive paths are gray even in Rainbow and Experimental modes.
    QLinearGradient bg(0, surface_rect.height(), 0, 0);
    updatePathGradient(bg, lateral_active);
    painter.setBrush(bg);
    painter.drawPolygon(track_vertices);
  }
}

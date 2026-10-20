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

namespace {
// LeadData carries no vehicle dimensions, so these describe a typical compact car rather than
// measuring one, which is what makes the outline land on the lead instead of being guessed at.
constexpr float LEAD_BOX_WIDTH = 1.8f;
constexpr float LEAD_BOX_HEIGHT = 1.5f;
// The reminder publishes its departure on a single frame; hold the box a moment longer.
constexpr uint64_t LEAD_BOX_DEPART_NANOS = 800000000ULL;
constexpr uint64_t LEAD_BOX_MAX_DATA_AGE_NANOS = 3000000000ULL;
constexpr float LEAD_BOX_PEN_WIDTH = 3.0f;
}  // namespace

void ModelRendererSP::drawLeadTrackingBox(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_one,
                                          const cereal::XYZTData::Reader &position, const QRect &surface_rect) {
  using LeadDepartState = cereal::SelfdriveStateSP::LeadDepartState;
  auto *s = uiState();
  const auto &sm = *s->sm;
  const uint64_t now = nanos_since_boot();

  // The box is only ever a view of the reminder's tracker, so with nothing fresh to read there
  // is nothing to draw, and no fade to carry over into the next stop. The tracker's message keeps
  // arriving for a while after radard stalls, and the box is projected from radarState rather than
  // from that message, so require both.
  const bool tracker_fresh = s->scene.started && sm.alive("selfdriveStateSP") && sm.valid("selfdriveStateSP") &&
                             sm.rcv_frame("selfdriveStateSP") > s->scene.started_frame &&
                             now - sm.rcv_time("selfdriveStateSP") <= LEAD_BOX_MAX_DATA_AGE_NANOS &&
                             sm.alive("radarState") && sm.valid("radarState");
  if (!s->scene.lead_tracking_box || !tracker_fresh) {
    lead_depart_nanos = 0;
    return;
  }

  const auto state = sm["selfdriveStateSP"].getSelfdriveStateSP().getLeadDepartState();
  const bool tracking = state == LeadDepartState::ACQUIRING || state == LeadDepartState::ARMED;
  if (state == LeadDepartState::DEPARTING) {
    lead_depart_nanos = now;
  } else if (tracking) {
    lead_depart_nanos = 0;
  }
  const bool departing = lead_depart_nanos != 0 && now - lead_depart_nanos <= LEAD_BOX_DEPART_NANOS;
  if (!tracking && !departing) {
    return;
  }

  const float d_rel = lead_one.getDRel();
  const int line_len = std::min(int(position.getX().size()), int(position.getZ().size()));
  // A dropped lead would leave a departure fade drawing around a stale range, and the reminder
  // only ever tracks leads whose status is set.
  if (!lead_one.getStatus() || !std::isfinite(d_rel) || d_rel <= 0.0f || line_len == 0) {
    return;
  }
  // Same ground height the chevron uses, so the box sits where the lead does.
  const int z_idx = std::min(get_path_length_idx(position, d_rel), line_len - 1);
  const float ground_z = position.getZ()[z_idx] + path_offset_z;
  // car_space_transform consumes the device frame: x forward, y right, z down (camera.py's
  // view_frame_from_device_frame). yRel is positive to the LEFT, so negate it exactly as
  // update_leads does. Some Hyundai SCC cars report no lateral offset at all, and the reminder
  // does not look at yRel, so guard it here instead.
  const float y_rel = std::isfinite(lead_one.getYRel()) ? -lead_one.getYRel() : 0.0f;
  // z grows downward, so the top of the outline is at a smaller z than the road.
  const float top_z = ground_z - LEAD_BOX_HEIGHT;

  // An upright outline standing at the lead's own range, sized like a compact car. Projecting it
  // rather than sizing it in image space is what keeps it the right size at every distance.
  QPointF corners[4];
  bool visible[4] = {};
  auto project = [&](int i, float y, float z) {
    const Eigen::Vector3f projected = car_space_transform * Eigen::Vector3f(d_rel, y, z);
    if (!projected.allFinite() || projected.z() <= 1e-3f) return;
    corners[i] = QPointF(projected.x() / projected.z(), projected.y() / projected.z());
    visible[i] = true;
  };
  const float half_width = LEAD_BOX_WIDTH / 2.0f;
  project(0, y_rel - half_width, ground_z);
  project(1, y_rel + half_width, ground_z);
  project(2, y_rel + half_width, top_z);
  project(3, y_rel - half_width, top_z);

  QColor color;
  if (departing) {
    const float remaining = 1.0f - float(now - lead_depart_nanos) / float(LEAD_BOX_DEPART_NANOS);
    color = QColor(144, 238, 168, int(255 * remaining));
  } else if (state == LeadDepartState::ARMED) {
    color = QColor(255, 165, 0, 230);
  } else {
    color = QColor(255, 255, 255, 150);
  }
  QPen pen(color, LEAD_BOX_PEN_WIDTH);
  pen.setCapStyle(Qt::RoundCap);
  pen.setJoinStyle(Qt::RoundJoin);

  painter.save();
  painter.setClipRect(surface_rect);
  painter.setPen(pen);
  painter.setBrush(Qt::NoBrush);
  static constexpr int EDGES[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
  for (const auto &[from, to] : EDGES) {
    if (visible[from] && visible[to]) {
      painter.drawLine(corners[from], corners[to]);
    }
  }
  painter.restore();
}

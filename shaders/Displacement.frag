#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  vec4 sample = texture(u_texture_3, uv);
  return sample.r;
  //return 0.0;
}

void main() {
  // YOUR CODE HERE
  vec3 b = cross(v_normal.xyz, v_tangent.xyz);
  mat3 tbn = mat3(v_tangent.xyz, b, v_normal.xyz);
  float h_uv = h(v_uv);
  vec2 v2_du = vec2((v_uv.x) + 1/u_texture_3_size.x, v_uv.y);
  vec2 v2_dv = vec2(v_uv.x, (v_uv.y) + 1/u_texture_3_size.y);
  float n_du = -1*((h(v2_du) - h_uv)*u_height_scaling*u_normal_scaling);
  float n_dv = -1*((h(v2_dv) - h_uv)*u_height_scaling*u_normal_scaling);
  vec3 n0 = vec3(n_du, n_dv, 1);
  vec3 nd = tbn * n0; //displaced normal

  //phong:
  vec3 l = u_light_pos.xyz - v_position.xyz;
  float r2 = length(l) * length(l);

  vec3 v = u_cam_pos - v_position.xyz;
  vec3 h = (v + l) / length(v + l);

  vec3 phong = (u_light_intensity / r2) * pow(max(0, dot(nd, normalize(h))), 80);
  vec3 diff = (u_light_intensity / r2) * max(0, dot(nd, normalize(l)));

  out_color.rgb = diff + phong;

  //normal testing:
  //out_color.rgb = (vec3(1, 1, 1) + nd) / 2;
  //out_color.a = 1;
  
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}


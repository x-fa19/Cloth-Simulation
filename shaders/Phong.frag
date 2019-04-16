#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float mag = length(v_position.xyz + u_cam_pos);
  //vec3 h = (v_position.xyz + u_cam_pos) / mag;

  vec3 l = u_light_pos.xyz - v_position.xyz;
  float r2 = length(l) * length(l);

  vec3 v = u_cam_pos - v_position.xyz;
  vec3 h = (v + l) / length(v + l);

  vec3 phong = 0.5 * (u_light_intensity / r2) * pow(max(0, dot(v_normal.xyz, normalize(h))), 40);
  vec3 diff = (u_light_intensity / r2) * max(0, dot(v_normal.xyz, normalize(l)));

  out_color.rgb = diff + phong;
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}


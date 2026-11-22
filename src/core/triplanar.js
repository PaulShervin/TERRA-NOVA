// Patches a MeshStandardMaterial to sample its `map` in WORLD space (triplanar) instead of per-object UVs.
// Result: textures keep a constant real-world scale (metres per tile) on any box size, no stretching, seamless
// across neighbouring instances. Also adds a cheap bump from the texture luminance and optional floor-dirt shading.
export function applyTriplanar(material, { tile = 3, bump = 0.5, dirt = false, roughnessVar = 0.35 } = {}) {
  material.onBeforeCompile = (shader) => {
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vTriPos;\nvarying vec3 vTriNrm;')
      .replace('#include <worldpos_vertex>', `#include <worldpos_vertex>
      {
        vec4 twp = vec4(transformed, 1.0);
        vec3 tn = objectNormal;
        #ifdef USE_INSTANCING
          twp = instanceMatrix * twp;
          tn = mat3(instanceMatrix) * tn;
        #endif
        twp = modelMatrix * twp;
        vTriPos = twp.xyz;
        vTriNrm = normalize(mat3(modelMatrix) * tn);
      }`);
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>
      varying vec3 vTriPos;
      varying vec3 vTriNrm;
      #define TRI_TILE ${tile.toFixed(2)}`)
      .replace('#include <map_fragment>', `
      vec3 triW = pow(abs(vTriNrm), vec3(6.0));
      triW /= (triW.x + triW.y + triW.z);
      vec4 sampledDiffuseColor = texture2D(map, vTriPos.zy / TRI_TILE) * triW.x
                               + texture2D(map, vTriPos.xz / TRI_TILE) * triW.y
                               + texture2D(map, vTriPos.xy / TRI_TILE) * triW.z;
      diffuseColor *= sampledDiffuseColor;
      float triLum = dot(sampledDiffuseColor.rgb, vec3(0.3333));
      ${dirt ? 'diffuseColor.rgb *= mix(vec3(0.5, 0.48, 0.46), vec3(1.0), smoothstep(0.0, 1.9, vTriPos.y));' : ''}`)
      .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
      roughnessFactor = clamp(roughnessFactor * (1.0 + ${roughnessVar.toFixed(2)} * (0.5 - triLum) * 2.0), 0.25, 1.0);`)
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
      {
        vec3 sdx = dFdx(-vViewPosition), sdy = dFdy(-vViewPosition);
        float hx = dFdx(triLum), hy = dFdy(triLum);
        vec3 r1 = cross(sdy, normal), r2 = cross(normal, sdx);
        float det = dot(sdx, r1);
        vec3 grad = sign(det) * (hx * r1 + hy * r2);
        normal = normalize(abs(det) * normal - ${bump.toFixed(2)} * grad * 0.6);
      }`);
  };
  material.customProgramCacheKey = () => `tri_${tile}_${bump}_${dirt}_${roughnessVar}`;
  material.needsUpdate = true;
  return material;
}

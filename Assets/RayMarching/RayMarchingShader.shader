Shader "Hidden/RayMarchingShader"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
    }
    SubShader
    {
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM

            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct inputv
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct inputf
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            inputf vert (inputv input)
            {
                inputf output;
                output.vertex = UnityObjectToClipPos(input.vertex);
                output.uv = input.uv;
                return output;
            }

            sampler2D _MainTex;
            sampler2D _CameraDepthTexture;

            uniform float4x4 ivp;
            uniform float time;

            #define STEPS 100
            #define EPSILON 0.01

            float3x3 RadiansToMatrix(float3 rad)
            {
                float cy = cos(rad.y);
                float sy = sin(rad.y);
                float cx = cos(rad.x);
                float sx = sin(rad.x);
                float cz = cos(rad.z);
                float sz = sin(rad.z);

                return float3x3
                (
                    cy * cz, (-cx * sz) + (sx * sy * cz), (sx * sz) + (cx * sy * cz), 
                    cy * sz, (cx * cz) + (sx * sy * sz), (-sx * cz) + (cx * sy * sz), 
                    -sy, sx * cy, cx * cy
                );
            }

            float Mandelbulb(float3 eye, float3 position, float3 rotation, float scale)
            {
                int iter = 16;
                float bailout = 2;
                float power = 8;

                float dr = 2;
                float r = 0;
                float3 p = (position - eye) / scale;

                float3 rad = float3(radians(rotation.x), radians(rotation.y), radians(rotation.z));
                float3x3 rotationMatrix = RadiansToMatrix(rad);
                p = mul(rotationMatrix, p);

                for (int i = 0; i < iter; i++)
                {
                    r = length(p);
                    if (r > bailout) break;
                    float theta = acos(p.z / r) * power;
                    float phi = atan2(p.y, p.x) * power;
                    dr = pow(r, power - 1.0) * power * dr + 1.0;
                    float zr = pow(r, power);
                    p = zr * float3(sin(theta) * cos(phi), sin(phi) * sin(theta), cos(theta));
                    p += (position - eye) / scale;
                    p = mul(rotationMatrix, p);
                }

                return 0.5 * log(r) * r / dr;
            }

            float Sphere(float3 position, float radius)
            {
                return length(position) - radius;
            }

            float Smooth(float d1, float d2, float k)
            {
                float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0);
                return lerp( d2, d1, h ) - k*h*(1.0-h);
            }

            float InfiniteSpheres(float3 eye, float3 position, float radius, float spacing)
            {
                position -= eye;
                float3 p = abs(position) - spacing * floor(abs(position) / spacing + 0.5);
                float d = length(p) - radius;
                d = abs(d) - radius;
                return d;
            }

            // kelp stuff
            float Box(float3 p, float3 c, float3 s)
            {
                float x = max
                (   p.x - c.x - float3(s.x / 2., 0, 0),
                    c.x - p.x - float3(s.x / 2., 0, 0)
                );

                float y = max
                (   p.y - c.y - float3(s.y / 2., 0, 0),
                    c.y - p.y - float3(s.y / 2., 0, 0)
                );
                
                float z = max
                (   p.z - c.z - float3(s.z / 2., 0, 0),
                    c.z - p.z - float3(s.z / 2., 0, 0)
                );

                float d = x;
                d = max(d,y);
                d = max(d,z);
                return d;
            }

            float Random(float seed)
            {
                return frac(sin(seed) * 43758.5453);
            }
            
            float WavingKelpStrand(float3 eye, float rot, float frequency, float amplitude)
            {
                // Apply twist to the input position along the y-axis
                float c = cos(rot * eye.y);
                float s = sin(rot * eye.y);
                float2x2 m = float2x2(c, -s, s, c);
                float3 q = float3(mul(float2(eye.x, eye.z), m), eye.y);

                // Add sinusoidal wave motion
                float wave = sin((eye.y + time) * frequency) * amplitude;
                float3 p = q + float3(wave, 0, 0);

                // Evaluate the Box SDF function
                float3 boxscale = float3(0.3, 0.02, 100); // y and z are swapped
                return Box(p, float3(0, 0, 0), boxscale);
            }

            float RepeatingKelp(float3 eye, float3 size, float separation)
            {
                float3 id = round(eye / separation);
                float3 o = sign(eye - separation * id);
                float d = 1e20;
                for (int k = 0; k < 2; k++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        for (int i = 0; i < 2; i++)
                        {
                            float3 rid = id + float3(i, j, k) * o;
                            rid = clamp(rid, -(size - 1.0) * 0.5, (size - 1.0) * 0.5);
                            float3 r = eye - separation * rid;
                            float sdf = WavingKelpStrand(r, 2, 1, 0.1);
                            d = min(d, sdf);
                        }
                    }
                } 
                return d;
            }
            
            float Map(float3 eye)
            {
                /*
                float seperation = 2;
                float3 repetition = float3(10, 0, 10);
                float3 id = round(eye / seperation);
                float3 repeye = eye - seperation * clamp(id, -repetition, repetition);
                return WavingKelpStrand(repeye, 1, 1, 0.1, id);
                */

                float seperation = 2;
                float3 repetition = float3(10, 0, 10);
                return RepeatingKelp(eye, repetition, seperation);
            }

            float3 calcNormal(in float3 p)
            {
                float h = EPSILON;
                float2 k = float2(1, -1);
                return normalize(k.xyy * Map(p + k.xyy * h) + k.yyx * Map(p + k.yyx * h) + k.yxy * Map(p + k.yxy * h) + k.xxx * Map(p + k.xxx * h));
            }

            float3 RayDir(float2 uv)
            {
                float2 ndc = (uv - 0.5) * 2.0;
                float4 clip = mul(ivp, float4(ndc, 1, 1));
                float3 direction = normalize(clip.xyz / clip.w);
                return direction;
            }

            float CalculateRealDepth(float2 uv, float raw)
            {
                float2 ndc = (uv - 0.5) * 2.0;
                float4 clip = mul(ivp, float4(ndc, 1 - raw, 1));
                float4 worldPos = mul(unity_CameraToWorld, mul(ivp, clip));
                return length(worldPos.xyz - _WorldSpaceCameraPos);
            }

            fixed4 frag (inputf input) : SV_Target
            {
                float4 current = tex2D(_MainTex, input.uv);

                // calc ray direction
                float3 rayDir = RayDir(input.uv);

                // calc real depth
                float4 viewDirection = mul(unity_CameraInvProjection, float4(input.uv * 2.0 - 1.0, 1.0, 1.0));
                float3 viewPos = (viewDirection.xyz / viewDirection.w) * Linear01Depth(tex2D(_CameraDepthTexture, input.uv).r);
                float realDepth = length(viewPos);

                // marching
                float lastDist = EPSILON;
                for (int i = 0; i < STEPS; i++)
                {
                    // march current ray
                    float3 rayPos = _WorldSpaceCameraPos + rayDir * lastDist;
                    float stepDist = Map(rayPos);
                    float fullDist = lastDist + stepDist;

                    // behind depth buffer
                    if (fullDist > realDepth) return current;

                    // hit object
                    if (stepDist < EPSILON)
                    {
                        float3 lightpos = normalize(float3(1, 0.6, -1) * 10000);
                        float3 albedo = float3(0.2, 0.4, 0.2);
                        float3 normal = calcNormal(rayPos);
                        float diffuse = max(0.0, dot(lightpos, normal));
                        float3 specular = pow(clamp(dot(lightpos, normal), 0.0, 1.0), 64.0) * float3(0.3, 0.3, 0.3);
                        return float4(albedo * (diffuse + 0.2) + specular, 1.0);
                    }

                    // increase ray distance
                    lastDist += stepDist;
                }

                // no hit
                return current;
            }

            ENDCG
        }
    }
}

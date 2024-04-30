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

            float RoundedBox(float3 eye, float3 scale, float smooth)
            {
                float3 q = abs(eye) - scale + smooth;
                return length(max(q, 0.0f)) + min(max(q.x, max(q.y, q.z)), 0.0f) - smooth;
            }

            float Twist(float3 eye, float rot)
            {
                float c = cos(rot * eye.y);
                float s = sin(rot * eye.y);
                float2x2 mat = float2x2(c, -s, s, c);
                float3 q = float3(mul(mat, eye.xz), eye.y);

                return RoundedBox(q, float3(0.2, 0.01, 10), 0.01);
            }

            float KelpStrand(float3 eye)
            {
                return Twist(eye, 1.4);
            }

            float waveKelpStrand(float3 eye, float rot, float frequency, float amplitude, float time)
            {
                // Apply twist to the input position along the y-axis
                float c = cos(rot * eye.y);
                float s = sin(rot * eye.y);
                float2x2 mat = float2x2(c, -s, s, c);
                float3 q = float3(mul(mat, eye.xz), eye.y);

                // Add sinusoidal wave motion to create the kelp strand effect, animating over time
                float wave = sin((eye.y + time) * frequency) * amplitude;
                float3 p = q + float3(0, wave, 0);

                // Evaluate the rounded box SDF function
                return RoundedBox(p, float3(0.2, 0.01, 10), 0.01);
            }

            float Map(float3 eye)
            {
                return waveKelpStrand(eye, 1, 1, 0.1, time);

                /*
                float f = 2.0;
                for (int i = 0; i < sphereAmount; i++)
                {
                    float3 p = float3(sphereData[i * 4], sphereData[i * 4 + 1], sphereData[i * 4 + 2]);
                    float r = sphereData[i * 4 + 3];
                    float s = Sphere(position - p, r);
                    f = Smooth(s, f, 0.4);
                }
                return f;
                */
                
                //return Mandelbulb(eye, float3(0, 0, 0), float3(0, 0, 0), 1);
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

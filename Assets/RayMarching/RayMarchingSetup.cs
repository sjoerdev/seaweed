using UnityEngine;

[ExecuteInEditMode]
public class RayMarchingSetup : MonoBehaviour
{
    [Header("Zeewier:")]
    public Vector2 repetition = new Vector2(5, 5);
    [Range(2, 16)] public float seperation = 4;
    [Range(0, 0.5f)] public float twisting = 0.2f;
    [Range(8, 1024)] public int seed = 98;

    [Header("Sub Surface Scattering:")]
    [Range(0f, 1f)] public float intensity = 0.5f;
    [Range(0f, 0.4f)] public float radius = 0.1f;

    [Header("Other:")]
    public Material material;

    private void Start()
    {
        Camera.main.depthTextureMode = DepthTextureMode.Depth;
    }
    
    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Matrix4x4 viewProjectionMatrix = Camera.main.projectionMatrix * Camera.main.worldToCameraMatrix;

        material.SetMatrix("ivp", viewProjectionMatrix.inverse);
        material.SetFloat("time", Time.time);

        material.SetVector("repetition", repetition);
        material.SetFloat("seperation", seperation);
        material.SetFloat("twisting", twisting);
        material.SetInt("kelpseed", seed);

        material.SetFloat("sss_intensity", intensity);
        material.SetFloat("sss_radius", radius);

        Graphics.Blit(source, destination, material);
    }
}
using UnityEngine;

[ExecuteInEditMode]
public class RayMarchingSetup : MonoBehaviour
{
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
        Graphics.Blit(source, destination, material);
    }
}
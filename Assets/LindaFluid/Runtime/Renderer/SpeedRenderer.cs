using UnityEngine;

namespace Linda.Fluid
{
	public class SpeedRenderer : IRenderer
	{
		[SerializeField] Material material;
		[SerializeField] float circleRadius = 0.05f;
		[SerializeField] float maxSpeed = 10f;
		[SerializeField][Range(0, 1)] float transparency = 0.5f;
		[SerializeField] Gradient speedColorMap = new Gradient()
		{
			colorKeys = new GradientColorKey[] { new GradientColorKey(Color.blue, 0f), new GradientColorKey(new Color(0.32f, 1.0f, 0.57f), 0.5f), new GradientColorKey(Color.yellow, 0.65f), new GradientColorKey(Color.red, 1f) },
			alphaKeys = new GradientAlphaKey[] { new GradientAlphaKey(1f, 1f), new GradientAlphaKey(1f, 1f), new GradientAlphaKey(1f, 1f), new GradientAlphaKey(1f, 1f) }
		};

		ComputeBuffer argsBuffer;
		Mesh mesh;

		public override void Initialize(ISimulation sim)
		{
			this.sim = sim;

			// set particle data buffers
			material.SetBuffer("positionBuffer", sim.devicePositionBuffer);
			material.SetBuffer("velocityBuffer", sim.deviceVelocityBuffer);

			// build and set a speed color texture
			Texture2D speedColorTexture = new Texture2D(256, 1, TextureFormat.RGBA32, false);
			for (int i = 0; i < speedColorTexture.width; ++i)
			{
				float t = i / (float)(speedColorTexture.width - 1);
				Color color = speedColorMap.Evaluate(t);
				speedColorTexture.SetPixel(i, 0, color);
			}
			speedColorTexture.Apply();
			material.SetTexture("speedColorMap", speedColorTexture);

			// build a quad mesh
			mesh = createQuadMesh();

			// build argument buffer for instancing
			uint[] args = new uint[5] { mesh.GetIndexCount(0), 0, 0, 0, 0 };
			argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
		}

		public override void Render()
		{
			uint[] args = new uint[5] { mesh.GetIndexCount(0), (uint)sim.numParticles, 0, 0, 0 };
			argsBuffer.SetData(args);

			material.SetFloat("radius", circleRadius);
			material.SetFloat("maxSpeed", maxSpeed);
			material.SetFloat("opacity", 1f - transparency);

			Graphics.DrawMeshInstancedIndirect(mesh, 0, material, new Bounds(Vector3.zero, Vector3.one * 1000f), argsBuffer);
		}

		public override void CleanUp()
		{
			argsBuffer?.Release();
		}
	}
}

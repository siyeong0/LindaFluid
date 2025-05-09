using Unity.Mathematics;
using UnityEngine;

namespace Linda.Fluid
{
	public class PICSolver : ISimulation
	{
		[Header("Simulation")]
		public float interactionRadius = 0.25f;
		public float targetDensity = 200f;
		public float pressureStiffness = 150f;
		public float nearPressureStiffness = 10f;
		public float viscosityStrength = 0.075f;
		public float relaxPositionRadius = 0.1f;
		public float relaxPositionStiffness = 0.01f;
		[Range(0, 1)] public float collisionDamping = 0.2f;
		public float gravity = 9f;
		public uint subStepCount = 4;
		public Bounds bounds;

		[Header("Input Interaction")]
		public float controlRadius = 3f;
		public float controlStregth = 35f;

		// compute buffers
		[HideInInspector] public ComputeBuffer deviceDensityBuffer;

		// compute shader kernels


		public override void Initialize()
		{
			base.Initialize();

			// set particle data
			float2[] positionBuffer = new float2[numParticles];
			float2[] velocityBuffer = new float2[numParticles];

			int numParcelsPerLine = Mathf.CeilToInt(Mathf.Sqrt(numParticles));
			float basePos = -numParcelsPerLine / 2 * spacing;
			for (int i = 0; i < numParticles; i++)
			{
				float x = i % numParcelsPerLine * spacing + basePos + initOffset.x;
				float y = i / numParcelsPerLine * spacing + basePos + initOffset.y;
				positionBuffer[i] = new float2(x, y);
				velocityBuffer[i] = float2.zero;
			}

			devicePositionBuffer.SetData(positionBuffer);
			deviceVelocityBuffer.SetData(velocityBuffer);
		}

		public override void Step()
		{
			updateColliders();
			updateSetting();
			updateBuffers();
			for (int i = 0; i < subStepCount; ++i)
			{
				simulationStep();
			}
		}

		public override void CleanUp()
		{
			base.CleanUp();
			deviceDensityBuffer?.Release();
		}

		void simulationStep()
		{
			// dispatch
		
		}

		void updateSetting()
		{
			float fixedDeltaTime = 1f / 120f;
			float predictDeltaTime = 1f / 240f;

			simulationComputeShader.SetFloat("deltaTime", fixedDeltaTime);
			simulationComputeShader.SetFloat("predictDeltaTime", predictDeltaTime);

			simulationComputeShader.SetFloat("interactionRadius", interactionRadius);
			simulationComputeShader.SetFloat("targetDensity", targetDensity);
			simulationComputeShader.SetFloat("pressureStiffness", pressureStiffness);
			simulationComputeShader.SetFloat("nearPressureStiffness", nearPressureStiffness);
			simulationComputeShader.SetFloat("relaxRadius", relaxPositionRadius);
			simulationComputeShader.SetFloat("relaxStiffness", relaxPositionStiffness);
			simulationComputeShader.SetFloat("viscosityStrength", viscosityStrength);

			simulationComputeShader.SetFloat("collisionDamping", collisionDamping);
			simulationComputeShader.SetVector("gravity", new Vector2(0, -gravity));
			simulationComputeShader.SetVector("bounds", new Vector4(-bounds.size.x, -bounds.size.y, bounds.size.x, bounds.size.y));
		}

		void updateBuffers()
		{
			simulationComputeShader.SetInt("numParticles", numParticles);

			(var inputPosition, var interactionInputStrength) = getMouseInput();
			simulationComputeShader.SetFloat("inputControlRadius", controlRadius);
			simulationComputeShader.SetVector("inputPosition", inputPosition);
			simulationComputeShader.SetFloat("inputStrength", interactionInputStrength);
		}

		(Vector2, float) getMouseInput()
		{
			bool bLeftButtonPressed = Input.GetMouseButton(0);
			bool bRightButtonPressed = Input.GetMouseButton(1);
			float inputStrength = 0f;
			Vector2 inputPosition = new Vector2(
					Camera.main.ScreenToWorldPoint(Input.mousePosition).x,
					Camera.main.ScreenToWorldPoint(Input.mousePosition).y);
			if (bLeftButtonPressed || bRightButtonPressed)
			{
				inputStrength += Input.GetMouseButton(0) ? controlStregth : 0f;
				inputStrength += Input.GetMouseButton(1) ? -controlStregth : 0f;
			}

			return (inputPosition, inputStrength);
		}
	}
}

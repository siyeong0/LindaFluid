using Unity.Mathematics;
using UnityEngine;

namespace Linda.Fluid
{
	public class PICSolver : ISimulation
	{
		[Header("Simulation")]
		public float cellSize = 0.25f;
		public Bounds bounds;
		public float targetDensity = 200f;
		public float pressureStiffness = 150f;
		public float nearPressureStiffness = 10f;
		public float viscosityStrength = 0.075f;
		public float relaxPositionRadius = 0.1f;
		public float relaxPositionStiffness = 0.01f;
		[Range(0, 1)] public float collisionDamping = 0.2f;
		public float gravity = 9f;
		public uint subStepCount = 4;

		[Header("Input Interaction")]
		public float controlRadius = 3f;
		public float controlStregth = 35f;

		// cell grid info
		Vector2Int gridSize;

		// compute buffers
		[HideInInspector] public ComputeBuffer devicePredictedPositionBuffer;
		[HideInInspector] public ComputeBuffer deviceDivergenceBuffer;
		[HideInInspector] public ComputeBuffer deviceDensityBuffer;

		[HideInInspector] public ComputeBuffer deviceSpatialEntryBuffer;
		[HideInInspector] public ComputeBuffer deviceSpatialOffsetBuffer;

		[HideInInspector] public ComputeBuffer deviceColliderPolygonPointBuffer;
		[HideInInspector] public ComputeBuffer deviceColliderPolygonOffsetBuffer;

		// spatial grid
		protected int numEntries;

		// compute shader kernels
		int applyExternalForcesKernel;
		int updateSpatialEntriesKernel;
		int sortKernel;
		int updateSpatialOffsetsKernel;
		int updateDensitiesKernel;
		int updatePositionsKernel;

		public override void Initialize()
		{
			base.Initialize();

			numEntries = getNextPow2(numParticles);

			// init kernels
			applyExternalForcesKernel = simulationComputeShader.FindKernel("ApplyExternalForces");
			updateSpatialEntriesKernel = simulationComputeShader.FindKernel("UpdateSpatialEntries");
			sortKernel = simulationComputeShader.FindKernel("Sort");
			updateSpatialOffsetsKernel = simulationComputeShader.FindKernel("UpdateSpatialOffsets");
			updateDensitiesKernel = simulationComputeShader.FindKernel("UpdateDensities");
			updatePositionsKernel = simulationComputeShader.FindKernel("UpdatePositions");

			// calc grid size
			gridSize.x = Mathf.CeilToInt(bounds.size.x / cellSize);
			gridSize.y = Mathf.CeilToInt(bounds.size.y / cellSize);

			// init device buffers
			devicePredictedPositionBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);
			deviceDivergenceBuffer = new ComputeBuffer(gridSize.x * gridSize.y, sizeof(float));
			deviceDensityBuffer = new ComputeBuffer(gridSize.x * gridSize.y, sizeof(float));

			deviceSpatialEntryBuffer = new ComputeBuffer(numEntries, sizeof(uint) * 3);
			deviceSpatialOffsetBuffer = new ComputeBuffer(numEntries, sizeof(uint));

			deviceColliderPolygonPointBuffer = new ComputeBuffer(maxColliderPolygonPoints, sizeof(float) * 2);
			deviceColliderPolygonOffsetBuffer = new ComputeBuffer(maxColliderPolygonPoints / 3, sizeof(uint));

			// set compute shader buffers
			simulationComputeShader.SetBuffer(applyExternalForcesKernel, "positionBuffer", devicePositionBuffer);
			simulationComputeShader.SetBuffer(applyExternalForcesKernel, "predictedPositionBuffer", devicePredictedPositionBuffer);
			simulationComputeShader.SetBuffer(applyExternalForcesKernel, "velocityBuffer", deviceVelocityBuffer);

			simulationComputeShader.SetBuffer(updateSpatialEntriesKernel, "predictedPositionBuffer", devicePredictedPositionBuffer);
			simulationComputeShader.SetBuffer(updateSpatialEntriesKernel, "spatialEntryBuffer", deviceSpatialEntryBuffer);
			simulationComputeShader.SetBuffer(updateSpatialEntriesKernel, "spatialOffsetBuffer", deviceSpatialOffsetBuffer);

			simulationComputeShader.SetBuffer(sortKernel, "spatialEntryBuffer", deviceSpatialEntryBuffer);

			simulationComputeShader.SetBuffer(updateSpatialOffsetsKernel, "spatialEntryBuffer", deviceSpatialEntryBuffer);
			simulationComputeShader.SetBuffer(updateSpatialOffsetsKernel, "spatialOffsetBuffer", deviceSpatialOffsetBuffer);

			simulationComputeShader.SetBuffer(updateDensitiesKernel, "predictedPositionBuffer", devicePredictedPositionBuffer);
			simulationComputeShader.SetBuffer(updateDensitiesKernel, "densityBuffer", deviceDensityBuffer);
			simulationComputeShader.SetBuffer(updateDensitiesKernel, "spatialEntryBuffer", deviceSpatialEntryBuffer);
			simulationComputeShader.SetBuffer(updateDensitiesKernel, "spatialOffsetBuffer", deviceSpatialOffsetBuffer);

			simulationComputeShader.SetBuffer(updatePositionsKernel, "positionBuffer", devicePositionBuffer);
			simulationComputeShader.SetBuffer(updatePositionsKernel, "velocityBuffer", deviceVelocityBuffer);
			simulationComputeShader.SetBuffer(updatePositionsKernel, "colliderPolygonPointBuffer", deviceColliderPolygonPointBuffer);
			simulationComputeShader.SetBuffer(updatePositionsKernel, "colliderPolygonOffsetBuffer", deviceColliderPolygonOffsetBuffer);

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
			devicePredictedPositionBuffer?.Release();
			deviceDivergenceBuffer?.Release();
			deviceDensityBuffer?.Release();
			deviceSpatialEntryBuffer?.Release();
			deviceSpatialOffsetBuffer?.Release();
			deviceColliderPolygonPointBuffer?.Release();
			deviceColliderPolygonOffsetBuffer?.Release();
		}

		void simulationStep()
		{
			// dispatch
			simulationComputeShader.Dispatch(applyExternalForcesKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
			simulationComputeShader.Dispatch(updateSpatialEntriesKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
			for (int k = 2; k <= numEntries; k *= 2)
			{
				for (int j = k / 2; j > 0; j /= 2)
				{
					simulationComputeShader.SetInt("k", k);
					simulationComputeShader.SetInt("j", j);
					simulationComputeShader.Dispatch(sortKernel, Mathf.CeilToInt(numEntries / 256f), 1, 1);
				}
			}
			simulationComputeShader.Dispatch(updateSpatialOffsetsKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
			simulationComputeShader.Dispatch(updateDensitiesKernel, Mathf.CeilToInt(gridSize.x / 32f), Mathf.CeilToInt(gridSize.y / 32f), 1);
			simulationComputeShader.Dispatch(updatePositionsKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
		}

		void updateSetting()
		{
			float fixedDeltaTime = 1f / 120f;
			float predictDeltaTime = 1f / 240f;

			simulationComputeShader.SetFloat("deltaTime", fixedDeltaTime);
			simulationComputeShader.SetFloat("predictDeltaTime", predictDeltaTime);

			simulationComputeShader.SetFloat("cellSize", cellSize);
			simulationComputeShader.SetVector("bounds", new Vector4(-bounds.size.x, -bounds.size.y, bounds.size.x, bounds.size.y));
			simulationComputeShader.SetInts("gridSize", new int[]{ gridSize.x, gridSize.y});

			simulationComputeShader.SetFloat("targetDensity", targetDensity);
			simulationComputeShader.SetFloat("pressureStiffness", pressureStiffness);
			simulationComputeShader.SetFloat("nearPressureStiffness", nearPressureStiffness);
			simulationComputeShader.SetFloat("relaxRadius", relaxPositionRadius);
			simulationComputeShader.SetFloat("relaxStiffness", relaxPositionStiffness);
			simulationComputeShader.SetFloat("viscosityStrength", viscosityStrength);

			simulationComputeShader.SetFloat("collisionDamping", collisionDamping);
			simulationComputeShader.SetVector("gravity", new Vector2(0, -gravity));
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

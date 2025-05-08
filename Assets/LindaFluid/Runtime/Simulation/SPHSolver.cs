using System.Linq;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;

namespace Linda.Fluid
{
	public class SPHSolver : ISimulation
	{
		// compute shader kernels
		int applyExternalForcesKernel;
		int updateSpatialEntriesKernel;
		int sortKernel;
		int updateSpatialOffsetsKernel;
		int updateDensitiesKernel;
		int relaxPositionsKernel;
		int applyPressureForcesKernel;
		int applyViscocityForcesKernel;
		int updatePositionsKernel;

		public override void Initialize()
		{
			base.Initialize();

			// init kernels
			applyExternalForcesKernel = simulationComputeShader.FindKernel("ApplyExternalForces");
			updateSpatialEntriesKernel = simulationComputeShader.FindKernel("UpdateSpatialEntries");
			sortKernel = simulationComputeShader.FindKernel("Sort");
			updateSpatialOffsetsKernel = simulationComputeShader.FindKernel("UpdateSpatialOffsets");
			updateDensitiesKernel = simulationComputeShader.FindKernel("UpdateDensities");
			relaxPositionsKernel = simulationComputeShader.FindKernel("RelaxPositions");
			applyPressureForcesKernel = simulationComputeShader.FindKernel("ApplyPressureForces");
			applyViscocityForcesKernel = simulationComputeShader.FindKernel("ApplyViscocityForces");
			updatePositionsKernel = simulationComputeShader.FindKernel("UpdatePositions");

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

			simulationComputeShader.SetBuffer(relaxPositionsKernel, "predictedPositionBuffer", devicePredictedPositionBuffer);
			simulationComputeShader.SetBuffer(relaxPositionsKernel, "spatialEntryBuffer", deviceSpatialEntryBuffer);
			simulationComputeShader.SetBuffer(relaxPositionsKernel, "spatialOffsetBuffer", deviceSpatialOffsetBuffer);

			simulationComputeShader.SetBuffer(applyPressureForcesKernel, "predictedPositionBuffer", devicePredictedPositionBuffer);
			simulationComputeShader.SetBuffer(applyPressureForcesKernel, "velocityBuffer", deviceVelocityBuffer);
			simulationComputeShader.SetBuffer(applyPressureForcesKernel, "densityBuffer", deviceDensityBuffer);
			simulationComputeShader.SetBuffer(applyPressureForcesKernel, "spatialEntryBuffer", deviceSpatialEntryBuffer);
			simulationComputeShader.SetBuffer(applyPressureForcesKernel, "spatialOffsetBuffer", deviceSpatialOffsetBuffer);

			simulationComputeShader.SetBuffer(applyViscocityForcesKernel, "predictedPositionBuffer", devicePredictedPositionBuffer);
			simulationComputeShader.SetBuffer(applyViscocityForcesKernel, "velocityBuffer", deviceVelocityBuffer);
			simulationComputeShader.SetBuffer(applyViscocityForcesKernel, "spatialEntryBuffer", deviceSpatialEntryBuffer);
			simulationComputeShader.SetBuffer(applyViscocityForcesKernel, "spatialOffsetBuffer", deviceSpatialOffsetBuffer);

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
			devicePredictedPositionBuffer.SetData(positionBuffer);
			deviceVelocityBuffer.SetData(velocityBuffer);

			// init spatial grid
			uint3[] spatialEntryBuffer = new uint3[numEntries];
			uint[] spatialOffsetBuffer = new uint[numEntries];
			for (int i = 0; i < numEntries; i++)
			{
				spatialEntryBuffer[i] = new uint3(uint.MaxValue, uint.MaxValue, uint.MaxValue);
				spatialOffsetBuffer[i] = (uint)numEntries;
			}

			deviceSpatialEntryBuffer.SetData(spatialEntryBuffer);
			deviceSpatialOffsetBuffer.SetData(spatialOffsetBuffer);
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
			devicePositionBuffer?.Release();
			deviceVelocityBuffer?.Release();
			devicePredictedPositionBuffer?.Release();
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
			simulationComputeShader.Dispatch(updateDensitiesKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
			simulationComputeShader.Dispatch(relaxPositionsKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
			simulationComputeShader.Dispatch(applyPressureForcesKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
			simulationComputeShader.Dispatch(applyViscocityForcesKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
			simulationComputeShader.Dispatch(updatePositionsKernel, Mathf.CeilToInt(numParticles / 1024f), 1, 1);
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
			simulationComputeShader.SetInt("numEntries", numEntries);

			simulationComputeShader.SetInt("numColliderPolygons", numColliderPolygons);
			deviceColliderPolygonPointBuffer.SetData(hostColliderPolygonPointBuffer);
			deviceColliderPolygonOffsetBuffer.SetData(hostColliderPolygonOffsetBuffer);

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

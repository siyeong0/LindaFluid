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
			numEntries = getNextPow2(numParticles);

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

			// init device buffers
			devicePositionBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);
			devicePredictedPositionBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);
			deviceVelocityBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);
			deviceDensityBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);

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
			updateConstants();
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

		void updateConstants()
		{
			float fixedDeltaTime = 1f / 120f;
			float predictDeltaTime = 1f / 240f;

			// update constant values
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

			(var inputPosition, var interactionInputStrength) = getMouseInput();
			simulationComputeShader.SetFloat("inputControlRadius", controlRadius);
			simulationComputeShader.SetVector("inputPosition", inputPosition);
			simulationComputeShader.SetFloat("inputStrength", interactionInputStrength);

			simulationComputeShader.SetInt("numParticles", numParticles);
			simulationComputeShader.SetInt("numEntries", numEntries);
		}

		public void updateColliders()
		{
			numColliderPolygons = 0;
			int pointCount = 0;
			int polygonCount = 0;

			Rigidbody2D[] rigidbodies = Object.FindObjectsByType<Rigidbody2D>(FindObjectsSortMode.None);
			foreach (var rigidbody in rigidbodies)
			{
				Collider2D[] colliders = rigidbody.GetComponentsInChildren<Collider2D>();
				CompositeCollider2D composite = colliders
					.OfType<CompositeCollider2D>()
					.FirstOrDefault();
				if (composite != null)
				{
					int pathCount = composite.pathCount;
					numColliderPolygons += pathCount;
					for (int i = 0; i < pathCount; i++)
					{
						int numPoints = composite.GetPathPointCount(i);
						Vector2[] points = new Vector2[numPoints];
						composite.GetPath(i, points);

						for (int j = 0; j < numPoints; j++)
						{
							hostColliderPolygonPointBuffer[pointCount++] = points[j];
						}
						hostColliderPolygonOffsetBuffer[polygonCount++] = (uint)pointCount;
					}
				}
				else
				{
					foreach (var collider in colliders)
					{
						if (collider is PolygonCollider2D poly)
						{
							numColliderPolygons += poly.pathCount;
							for (int i = 0; i < poly.pathCount; i++)
							{
								Vector2[] path = poly.GetPath(i);

								for (int j = 0; j < path.Length; j++)
								{
									hostColliderPolygonPointBuffer[pointCount++] = path[j];
								}
								hostColliderPolygonOffsetBuffer[polygonCount++] = (uint)pointCount;
							}
						}
						else if (collider is BoxCollider2D box)
						{
							Vector2 size = box.size * 0.5f;
							Vector2 offset = box.offset;
							Vector2[] localPath = new Vector2[] {
							offset + new Vector2( size.x, -size.y),
							offset + new Vector2( size.x,  size.y),
							offset + new Vector2(-size.x,  size.y),
							offset + new Vector2(-size.x, -size.y),
						};
							Vector2[] path = localPath.Select(p => (Vector2)box.transform.TransformPoint(p)).ToArray();

							++numColliderPolygons;
							for (int j = 0; j < path.Length; j++)
							{
								hostColliderPolygonPointBuffer[pointCount++] = path[j];
							}
							hostColliderPolygonOffsetBuffer[polygonCount++] = (uint)pointCount;
						}
						else if (collider is CircleCollider2D circle)
						{
							int segments = 20;
							Vector2[] localPath = new Vector2[segments];
							for (int i = 0; i < segments; i++)
							{
								float angle = Mathf.Deg2Rad * 360f / segments * i;
								localPath[i] = circle.offset + new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * circle.radius;
							}
							Vector2[] path = localPath.Select(p => (Vector2)circle.transform.TransformPoint(p)).ToArray();

							++numColliderPolygons;
							for (int j = 0; j < path.Length; j++)
							{
								hostColliderPolygonPointBuffer[pointCount++] = path[j];
							}
							hostColliderPolygonOffsetBuffer[polygonCount++] = (uint)pointCount;
						}
						else if (collider is EdgeCollider2D edge)
						{
							Vector2[] path = edge.points;

							++numColliderPolygons;
							for (int j = 0; j < path.Length; j++)
							{
								hostColliderPolygonPointBuffer[pointCount++] = path[j];
							}
							hostColliderPolygonOffsetBuffer[polygonCount++] = (uint)pointCount;
						}
						else
						{
							Debug.Assert(false, "Invalide collider type");
						}
					}
				}
			}

			// update compute buffers
			simulationComputeShader.SetInt("numColliderPolygons", numColliderPolygons);
			deviceColliderPolygonPointBuffer.SetData(hostColliderPolygonPointBuffer);
			deviceColliderPolygonOffsetBuffer.SetData(hostColliderPolygonOffsetBuffer);
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

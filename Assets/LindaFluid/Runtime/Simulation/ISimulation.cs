using System.Linq;
using Unity.Mathematics;
using UnityEngine;

namespace Linda.Fluid
{
	public abstract class ISimulation
	{
		[Header("Comput Shader")]
		public ComputeShader simulationComputeShader;

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

		[Header("Initial Placement")]
		public int numParticles = 20000;
		public float spacing = 0.01f;
		public Vector2 initOffset;

		// compute buffers
		[HideInInspector] public ComputeBuffer devicePositionBuffer;
		[HideInInspector] public ComputeBuffer devicePredictedPositionBuffer;
		[HideInInspector] public ComputeBuffer deviceVelocityBuffer;

		[HideInInspector] public ComputeBuffer deviceSpatialEntryBuffer;
		[HideInInspector] public ComputeBuffer deviceSpatialOffsetBuffer;

		[HideInInspector] public ComputeBuffer deviceColliderPolygonPointBuffer;
		[HideInInspector] public ComputeBuffer deviceColliderPolygonOffsetBuffer;

		// spatial grid
		protected int numEntries;

		// collision
		protected const int maxColliderPolygonPoints = 4096;
		protected int numColliderPolygons;
		protected float2[] hostColliderPolygonPointBuffer;
		protected uint[] hostColliderPolygonOffsetBuffer;


		public virtual void Initialize()
		{
			numEntries = getNextPow2(numParticles);

			// init device buffers
			devicePositionBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);
			devicePredictedPositionBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);
			deviceVelocityBuffer = new ComputeBuffer(numParticles, sizeof(float) * 2);

			deviceSpatialEntryBuffer = new ComputeBuffer(numEntries, sizeof(uint) * 3);
			deviceSpatialOffsetBuffer = new ComputeBuffer(numEntries, sizeof(uint));

			deviceColliderPolygonPointBuffer = new ComputeBuffer(maxColliderPolygonPoints, sizeof(float) * 2);
			deviceColliderPolygonOffsetBuffer = new ComputeBuffer(maxColliderPolygonPoints / 3, sizeof(uint));

			// init host buffers
			hostColliderPolygonPointBuffer = new float2[maxColliderPolygonPoints];
			hostColliderPolygonOffsetBuffer = new uint[maxColliderPolygonPoints / 3];
		}

		public abstract void Step();
		public abstract void CleanUp();

		protected void updateColliders()
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
		}

		protected int getNextPow2(int n)
		{
			int power = 1;
			while (power < n)
				power *= 2;
			return power;
		}
	}
}

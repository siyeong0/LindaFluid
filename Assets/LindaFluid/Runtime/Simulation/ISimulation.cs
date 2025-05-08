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
		[HideInInspector] public ComputeBuffer deviceDensityBuffer;

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


		public abstract void Initialize();
		public abstract void Step();
		public abstract void CleanUp();

		protected int getNextPow2(int n)
		{
			int power = 1;
			while (power < n)
				power *= 2;
			return power;
		}
	}
}

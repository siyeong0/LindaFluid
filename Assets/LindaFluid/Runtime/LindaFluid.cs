using Unity.VisualScripting;
using UnityEngine;

namespace Linda
{
    public class LindaFluid : MonoBehaviour
    {
		[SerializeReference, SubclassPicker] Fluid.ISimulation simulation;
		[Space(10)]
		[SerializeReference, SubclassPicker] Fluid.IRenderer rendering;

		private void Start()
        {
            simulation.Initialize();
            rendering.Initialize(simulation);
        }

		private void Update()
        {
            rendering.Render();
        }

		private void FixedUpdate()
		{
			simulation.Step();
		}

		private void OnDestroy()
		{
			simulation.CleanUp();
			rendering.CleanUp();
		}
	}
}
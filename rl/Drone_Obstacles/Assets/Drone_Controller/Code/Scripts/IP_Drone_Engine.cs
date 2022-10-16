using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace gokul 
{   
    [RequireComponent(typeof(BoxCollider))]
    public class IP_Drone_Engine : IEngine
    {
        // Start is called before the first frame update
       #region Variables
       [Header("Engine Properties")]
       [SerializeField] private float maxPower = 4f;
       #endregion

     
        
        #region Interface Methods
        public void InitEngine()
        {
            throw new System.NotImplementedException();
        }
        
        public void UpdateEngine()
        {
            throw new System.NotImplementedException();
        }
        #endregion
    }
}

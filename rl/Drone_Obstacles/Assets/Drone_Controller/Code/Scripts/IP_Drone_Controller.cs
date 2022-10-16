using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace gokul 
{
    [RequireComponent(typeof(IP_Drone_Inputs))]
    
    public class IP_Drone_Controller : IP_Base_Rigidbody
    {
        #region Variables
        [Header("Control Properties")]
        [SerializeField]private float minMaxPitch = 30f;
        [SerializeField]private float minMaxRoll = 30f;
        [SerializeField]private float yawPower = 4f;
        
        
        private IP_Drone_Inputs input;
        
        #endregion
        
        #region Main Methods
        // Start is called before the first frame update
        void Start()
        {
            input = GetComponent<IP_Drone_Inputs>();
            
        }
        #endregion
        
        #region Custom Methods 
        protected override void HandlePhysics(){
            HandleEngines();
            HandleControls();
            
        }
        
        protected virtual void HandleControls()
        {
            rb.AddForce(Vector3.up*(rb.mass*Physics.gravity.magnitude));
        }
        
        protected virtual void HandleEngines()
        {
        
        }
        #endregion
       

       
    }
  }

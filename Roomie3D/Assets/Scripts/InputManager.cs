using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.ARFoundation;

public class InputManager : MonoBehaviour
{
    //reperance to the object
    //[SerializeField] private GameObject arObj;
    [SerializeField] private Camera arCam;
    [SerializeField] private ARRaycastManager _raycastManager;

    private List<ARRaycastHit> _hits = new List<ARRaycastHit>();
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetMouseButtonDown(0))
        {
            //if the user touches the object we will get an array to the position
            Ray ray = arCam.ScreenPointToRay(Input.mousePosition);
            if(_raycastManager.Raycast(ray,_hits))
            {
                Pose pose = _hits[0].pose;
                //Instantiate(DataHandler.Instance.funiture, pose.position, pose.rotation);
            }
        }
    }
}

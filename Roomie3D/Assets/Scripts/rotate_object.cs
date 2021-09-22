using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class rotate_object : MonoBehaviour
{
    public GameObject ObjectRotate;
    public float rotateSpeed = 50f;
    bool rotateStatus = false;

    public void Rotasi()
    {
        if(rotateStatus == false)
        {
            rotateStatus = true;
        }
        else
        {
            rotateStatus = false;
        }
    }

    void update()
    {
        if(rotateStatus == true)
        {
            ObjectRotate.transform.Rotate(Vector3.up, rotateSpeed + Time.deltaTime);
        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class scale_object : MonoBehaviour
{
    public GameObject Object;
    private bool Zoomin;
    private bool Zoomout;

    public float Scale = 0.1f;

    // Update is called once per frame
    void Update()
    {
        if (Zoomin)
        {
            Object.transform.localScale += new Vector3(Scale, Scale, Scale);

        }
        if (Zoomout)
        {
            Object.transform.localScale -= new Vector3(Scale, Scale, Scale);
        }

    }

    //make object scale big
    public void OnPressedZoomIn()
    {
        Zoomin = true;
    }
    
    public void OnReleasedZoomIn()
    {
        Zoomin = false;
    }
    //make object scale small
    public void OnPressedZoomOut()
    {
        Zoomout = true;
    }

    public void OnReleasedZoomOut()
    {
        Zoomout = false;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class create_object : MonoBehaviour
{
    public Transform spawnPos;
    public GameObject spawnee;

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButton(0))
        {
            Instantiate(spawnee, spawnPos.position, spawnPos.rotation);
        }
        
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class load_2_scene : MonoBehaviour
{
    [SerializeField]
    private float delayBeforeLoading = 10f;
    [SerializeField]
    private string sceneNameToLoad;
    //keeps track of how much time elapsed
    private float timeElapsed;

    public void Update()
    {
        //add the time that passed.
        timeElapsed += Time.deltaTime;

        if (timeElapsed > delayBeforeLoading)
        {
            //if the time had passed load the new scene with the right name.
            SceneManager.LoadScene(sceneNameToLoad);
        }
    }
}

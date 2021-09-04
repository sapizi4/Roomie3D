using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class chnge_scene : MonoBehaviour
{
    public void btn_change_scene(string scene_name)
    {
        SceneManager.LoadScene(scene_name);
    }
    
}

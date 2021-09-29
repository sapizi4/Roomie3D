using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class LoadRandomScene : MonoBehaviour
{
    public void LoadRandomScenes()
    {
        //by using this.random method to load random scenes from the quiz
        int index = Random.Range(12, 21);
        SceneManager.LoadScene(index);
    }
}

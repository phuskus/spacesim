using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{
    private SpriteRenderer sr;
    // Start is called before the first frame update
    void Start()
    {   
        sr = GetComponent<SpriteRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyUp(KeyCode.Space))
        {
            Vector3 center = sr.bounds.center;
            Vector3 extents = sr.bounds.extents;
            Vector3 min = sr.bounds.min;
            Vector3 max = sr.bounds.max;

            Debug.Log("Center: " + center);
            Debug.Log("Extents: " + extents);
            Debug.Log("Min: " + min);
            Debug.Log("Max: " + max);
        }
        
    }
}

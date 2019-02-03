
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class CellestialPhysics : MonoBehaviour
{
    // Physics variables
    public float mass, radius;
    private Vector2 force;
    private Vector2 acceleration;
    private Vector2 speed;
    

    void Awake()
    {
        speed = new Vector2(0, 0);
        acceleration = new Vector2(0, 0);
        force = new Vector2(0, 0);
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    public void SetImageScale(int KmPerSquare)
    {
        SpriteRenderer sr = GetComponent<SpriteRenderer>();
        float circleRadius = KmPerSquare / 2; // Since the circle sprite width is equal to grid square width
        float scale = radius / circleRadius;

        transform.localScale = new Vector3(scale, scale, 0);
    }

    public void SetSpeed(Vector2 sp)
    {
        speed = sp; // Expressed in km/s
    }
    public void CalculateAcceleration()
    {
        acceleration = force / mass; // Expressed in km/s^2
    }

    public void Accelerate(float seconds)
    {
        speed += seconds * acceleration;
    }

    public void Move(float seconds, int KmPerSquare)
    {
        transform.position += (new Vector3(speed.x * seconds, speed.y * seconds, 0)) / KmPerSquare; // Divided by KmPerSquare to convert to transform component position
    }
    public void ApplyForce(Vector2 f)
    {
        // Add a vector to the local force vector
        force += f; // Expressed in YN
    }

    public void ResetForce()
    {
        // Set force vector to zero
        force = new Vector2(0, 0);
    }

    void FixedUpdate()
    {

    }
    // Update is called once per frame
    void Update()
    {
    }
}

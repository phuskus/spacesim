using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;

public class DuplicateKeyComparer<TKey>
// Just an ICompare to regulate duplicate keys for the sorted list
                :
             IComparer<TKey> where TKey : IComparable
{
    #region IComparer<TKey> Members

    public int Compare(TKey x, TKey y)
    {
        int result = x.CompareTo(y);

        if (result == 0)
            return 1;   // Handle equality as beeing greater
        else
            return result;
    }

    #endregion
}
public class Pair<T> : IEquatable<Pair<T>>
{
    // Defines a two-object collection generic
    public T first, second;
    public Pair(T f, T s) {
        first = f;
        second = s;
    }

    public bool Equals(Pair<T> pair)
    {
        if((first.Equals(pair.first) && second.Equals(pair.second)) || (first.Equals(pair.second) && second.Equals(pair.first)))
        {
            return true;
        }
        return false;
    }
}


public enum ProjType {BEGIN, END}; // BEGIN - "left-most" point of a projection, END - "right-most" point of a projection
public class Projection : IEquatable<Projection>
{
    // Defines the projection of a support point onto an axis
    // (for use in collision detection)
    public float value;
    public CellestialPhysics body;
    public ProjType type;

    public Projection(float val, CellestialPhysics obj, ProjType t)
    {
        value = val;
        body = obj;
        type = t;
    }

    public Projection()
    {
        value = 0;
        body = null;
        type = ProjType.BEGIN;
    }

    public bool Equals(Projection p)
    {
        if(p.value == value && p.body.Equals(body) && p.type == type)
        {
            return true;
        }
        return false;
    }
}

public class ObjectManager : MonoBehaviour
{
    // Moon panel input boxes
    public InputField input_moon_mass;
    public InputField input_moon_radius;
    public InputField input_moon_distance;

    // Planet panel input boxes
    public InputField input_planet_mass;
    public InputField input_planet_radius;

    // Time speed slider
    public Slider slider_time;

    // Prefabs
    public CellestialPhysics moonPrefab;
    public CellestialPhysics planetPrefab;

    // Planet object
    public CellestialPhysics planet;

    // Collection of all cellestial objects in the simulation
    // except for the planet
    public List<CellestialPhysics> cellObjs;

    // List of all 2-element combinations of cellestial objects in the scene
    // - For more efficient force application (no duplicates, code runs twice as fast)
    private List<Pair<CellestialPhysics>> objectCombos;

    // Some simulation variables
    public int KilometersPerSquare;
    public float timeSpeed;


    public void SetTimeSpeed()
    {
        timeSpeed = slider_time.value;
    }
    void Awake()
    {
        // Initialization
        objectCombos = new List<Pair<CellestialPhysics>>();
        cellObjs = new List<CellestialPhysics>();

        // Instantiate the earth in the middle of the scene
        planet = (CellestialPhysics) Instantiate(planetPrefab, new Vector3(0, 0, 0), Quaternion.identity);
        planet.mass = 5972.6f;
        planet.radius = 6371;
        planet.SetImageScale(KilometersPerSquare);
    }
    // Start is called before the first frame update
    void Start()
    {
        // Instantiate The Moon
        AddMoon(73.4767309f, 1737, 384400);

        // Fill planet input boxes with values
        input_planet_mass.text = Convert.ToString(planet.mass);
        input_planet_radius.text = Convert.ToString(planet.radius);

        // Set slider value to initial value
        slider_time.value = timeSpeed;
    }

    Vector2 CalculateGravitationalForce(CellestialPhysics o1, CellestialPhysics o2)
    {
        // Calculates the gravitational force between two cellestial objects
        // return: Vector2 force - the grav. force vector pointing from o1 to o2
        float mass1 = o1.mass;
        float mass2 = o2.mass;
        Vector2 distanceVector = o2.transform.position - o1.transform.position;
        float distance = distanceVector.magnitude * KilometersPerSquare;

        float gravScalar = 66.742f * (mass1 * mass2) / (distance*distance); // Gravitational force formula (result is in YN)

        Vector2 force = distanceVector.normalized * gravScalar;
        return force;
    }

    Projection FindProjection(ProjType type, CellestialPhysics obj, Vector3 axis)
    {
        // Finds min (ProjType.BEGIN) or max (ProjType.END) projection of
        // an object on the provided axis
        // -- mark the axis you're interested in with a 1 for that vector
        //    component, leave all others on 0, i.e (0,1,0) checks the Y axis
        Projection proj = new Projection();
        proj.body = obj;

        SpriteRenderer sr = obj.GetComponent<SpriteRenderer>();
        float num;
        switch(type)
        {
            case ProjType.BEGIN:
                num = Vector3.Dot(axis, sr.bounds.min);

                proj.value = num;
                proj.type = ProjType.BEGIN;
                break;
            case ProjType.END:
                num = Vector3.Dot(axis, sr.bounds.max);
                proj.value = num;
                proj.type = ProjType.END;
                break;
        }

        return proj;
    }
    HashSet<Pair<CellestialPhysics>> GetCollidingObjectsOnAxis(Vector3 axis)
    {
        HashSet<Pair<CellestialPhysics>> ret = new HashSet<Pair<CellestialPhysics>>();
        // -- SORT --
        SortedList<float, Projection> projections = new SortedList<float, Projection>(new DuplicateKeyComparer<float>());
        // Find beginning and ending projection points for each body and
        // add them to the sorted list
        Projection b, e; // b - beggining proj, e - ending proj
        foreach(CellestialPhysics obj in cellObjs)
        {
            b = FindProjection(ProjType.BEGIN, obj, axis);
            e = FindProjection(ProjType.BEGIN, obj, axis);

            projections.Add(b.value, b);
            projections.Add(e.value, e);
        }

        // -- SWEEP --
        List<CellestialPhysics> activeObjects = new List<CellestialPhysics>();
        foreach(Projection p in projections.Values)
        {
            switch(p.type)
            {
                case ProjType.BEGIN:
                    // If it's the left-most point, make the object active
                    activeObjects.Add(p.body);
                break;

                case ProjType.END:
                    // Deactivate its object
                    activeObjects.Remove(p.body);
                    // Pair this object with every other active one
                    // Since all active objects are currently
                    // colliding (maybe)
                    foreach(CellestialPhysics obj in activeObjects)
                    {
                        ret.Add(new Pair<CellestialPhysics>(p.body, obj));
                    }
                break;
            }
        }
        return ret;
    }

    HashSet<Pair<CellestialPhysics>> GetCollidingObjects()
    {   
        // Gets a list of object-pairs that might be colliding judging 
        // by their projections onto the axes of the space (The SORT AND SWEEP/PRUNE ALGORITHM)
        // RETURN: list of pairs of collision candidates
        HashSet<Pair<CellestialPhysics>> OnXAxis;
        HashSet<Pair<CellestialPhysics>> OnYAxis;

        Vector3 axis; // the axis on which we are sorting and sweeping
        
        // First we check along the X axis
        axis = new Vector3(1,0,0);
        OnXAxis = GetCollidingObjectsOnAxis(axis);

        // Then we check along the Y axis
        axis = new Vector3(0,1,0);
        OnYAxis = GetCollidingObjectsOnAxis(axis);

        // Intersect the two sets to extract objects that collide on both axes
        OnXAxis.IntersectWith(OnYAxis);

        return OnXAxis;
    }

    bool ResolveCollision(Pair<CellestialPhysics> pair)
    {
        // TODO: Implement resolution of two objects colliding

        // Checks if there is really a collision between a 
        // pair of bodies and resolves it if there is one
        // RETURN: true if they were colliding, false if they weren't

        // TEST CASE: Assuming all bodies are circles is a good start
        Vector3 center1 = pair.first.transform.position;
        Vector3 center2 = pair.second.transform.position;

        if((center2 - center1).magnitude * KilometersPerSquare < pair.first.radius + pair.second.radius)
        {
            // A collision happened, resolve it

            return true;
        }

        return false;
    }
    void FixedUpdate()
    {
        // --GRAVITATIONAL FORCE APPLICATION--
        // Reset all cellestial object forces
        planet.ResetForce();
        foreach(CellestialPhysics o in cellObjs)
        {
            o.ResetForce();
        }


        // Calculate and apply gravitational force for each 2-cellestial object combination
        Vector2 gravitationalForce;
        foreach(Pair<CellestialPhysics> combo in objectCombos)
        {
            gravitationalForce = CalculateGravitationalForce(combo.first, combo.second);
            combo.first.ApplyForce(gravitationalForce);
            combo.second.ApplyForce(-gravitationalForce);
        }

        // --ACCELERATION AND MOVEMENT--
        // Calculate acceleration vector of each cellestial object based on force
        // add it to speed (accelerate)
        // move the object based on time elapsed from last frame
        // PLANET IS NOT MOVED TO LOCK IT IN PLACE

        foreach(CellestialPhysics obj in cellObjs)
        {
            obj.CalculateAcceleration();
            obj.Accelerate(Time.deltaTime * timeSpeed);
            obj.Move(Time.deltaTime * timeSpeed, KilometersPerSquare);
        }

        // TODO: Implement collision checking

        // --COLLISION CHECKING--
        // While a collision has been found, keep checking and resolving
        //     1. Broad phase: Get a List of pairs of objects that are possibly colliding
        //     2. Narrow phase: Check if there really is a collision and resolve it for each collision candidate pair

        bool collisionFound = true;
        while(collisionFound)
        {
            collisionFound = false; // Presume a collision has not been found
            // BROAD PHASE
            HashSet<Pair<CellestialPhysics>> candidates = GetCollidingObjects();

            // NARROW PHASE
            foreach(Pair<CellestialPhysics> pair in candidates)
            {
                // If they are really colliding, resolve it
                collisionFound = ResolveCollision(pair);
            }
        }
        


    }


    private void AddMoon(float mass, float radius, float distance)
    {
        // Instantiate a moon and set appropriate variables
        CellestialPhysics obj = Instantiate(moonPrefab, planet.transform.position + new Vector3(distance / KilometersPerSquare, 0, 0), Quaternion.identity);
        obj.mass = mass;
        obj.radius = radius;
        obj.SetImageScale(KilometersPerSquare);

        // Apply starting speed perpendicular to gravitational force to achieve stable orbit
        Vector2 distanceVector = obj.transform.position - planet.transform.position;
        float speed = (float) Math.Sqrt(66.742f * planet.mass / distance);

        Vector2 directionVector = new Vector2(distanceVector.y, -distanceVector.x);
        obj.SetSpeed(directionVector.normalized * speed);

        // Add moon object to moons list
        cellObjs.Add(obj);

        Pair<CellestialPhysics> temp;
        // Add all new moon-moon combinations
        for(int i = cellObjs.Count-2; i >= 0; i--)
        {
            temp = new Pair<CellestialPhysics>(obj, cellObjs[i]);
            objectCombos.Add(temp);
        }
        // Plus one planet-moon combination
        temp = new Pair<CellestialPhysics>(planet, obj);
        objectCombos.Add(temp);

        // Log what's been done
        Debug.Log(String.Format("Added new moon (mass: {0} Yg, radius: {1} km, distance: {2} km)", mass, radius, distance));
    }
    public void AddMoon()
    {
        // Get data from text fields
        float mass, radius, distance;
        mass =  (float) Convert.ToDouble(input_moon_mass.text);
        radius = (float) Convert.ToDouble(input_moon_radius.text);
        distance = (float) Convert.ToDouble(input_moon_distance.text);
        
        // Instantiate moon
        AddMoon(mass, radius, distance);
    }

    public void UpdatePlanet()
    {
        if(planet != null)
        {
            // Update planet variables from input boxes
            // Set planet vars from input boxes
            planet.mass = (float) Convert.ToDouble(input_planet_mass.text);
            planet.radius = (float) Convert.ToDouble(input_planet_radius.text);

            // Resize planet sprite
            planet.SetImageScale(KilometersPerSquare);

            // Log what's been done
            Debug.Log(string.Format("Updated planet (mass: {0} Yg, radius: {1} km)", planet.mass, planet.radius));

        }
    }
}

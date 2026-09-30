using UnityEngine;

public class TestMovement : MonoBehaviour
{
    public Vector3 movementSpeed;
    public Vector3 rotationSpeed;

    private Vector3 _center;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _center = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = _center + Vector3.Scale(new Vector3(
            Mathf.Sin(Time.time),
            Mathf.Cos(Time.time),
            Mathf.Sin(Time.time)
        ), movementSpeed);

        transform.rotation = Quaternion.Euler(Time.time * rotationSpeed);
    }
}

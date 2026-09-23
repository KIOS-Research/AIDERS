/*
   Package 'cfc' (Camera Footprint Calculator) provides functionality to
   calculate the geographic area visible from a camera.
   It computes the bounding polygon of the visible area on the Earth's surface,
   taking into account the camera's:
       - position (latitude and longitude)
       - altitude, orientation (roll, pitch, heading)
       - horizontal and vertical field of view
*/

package cfc

import (
	"math"
)

/*
   // Example parameters for GetBoundingPolygon
   latitude := 40.7128 // New York City latitude
   longitude := -74.0060 // New York City longitude
   FOVh := 90.0 // Horizontal field of view in degrees
   FOVv := 60.0 // Vertical field of view in degrees
   altitude := 100.0 // Altitude in meters
   roll := 0.0 // Roll in degrees
   pitch := 0.0 // Pitch in degrees
   heading := 0.0 // Heading in degrees

   polygon := GetBoundingPolygon(latitude, longitude, FOVh, FOVv, altitude, roll, pitch, heading)
   fmt.Println("Bounding Polygon Coordinates:")
   for _, coord := range polygon {
       fmt.Printf("Latitude: %f, Longitude: %f\n", coord[0], coord[1])
   }
*/

// Vector struct represents a 3D vector.
type Vector struct {
	x, y, z float64
}

// Creates a new Vector.
func newVector(x, y, z float64) *Vector {
	return &Vector{x: x, y: y, z: z}
}

// Normalize the vector.
func (v *Vector) normalize() *Vector {
	mag := math.Sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
	return newVector(v.x/mag, v.y/mag, v.z/mag)
}

// GetBoundingPolygon calculates the bounding polygon of the camera's view.
func GetBoundingPolygon(latitude, longitude, FOVh, FOVv, altitude, roll, pitch, heading float64) [][2]float64 {
	ray11 := ray1(FOVh, FOVv)
	ray22 := ray2(FOVh, FOVv)
	ray33 := ray3(FOVh, FOVv)
	ray44 := ray4(FOVh, FOVv)

	rotatedVectors := rotateRays(ray11, ray22, ray33, ray44, roll, pitch, heading)

	origin := newVector(0, 0, altitude)
	intersections := getRayGroundIntersections(rotatedVectors, origin)

	var polygon [][2]float64
	for _, intersection := range intersections {
		shiftLatitude := intersection.x / 111139
		shiftLongitude := intersection.y / (111139 * math.Cos(degreesToRadians(latitude)))
		newLatitude := latitude + shiftLatitude
		newLongitude := longitude + shiftLongitude
		polygon = append(polygon, [2]float64{newLatitude, newLongitude})
	}

	return polygon
}

// ray1 calculates the first ray-vector.
func ray1(FOVh, FOVv float64) *Vector {
	ray := newVector(math.Tan(FOVv/2), math.Tan(FOVh/2), -1)
	return ray.normalize()
}

// ray2 calculates the second ray-vector.
func ray2(FOVh, FOVv float64) *Vector {
	ray := newVector(math.Tan(FOVv/2), -math.Tan(FOVh/2), -1)
	return ray.normalize()
}

// ray3 calculates the third ray-vector.
func ray3(FOVh, FOVv float64) *Vector {
	ray := newVector(-math.Tan(FOVv/2), -math.Tan(FOVh/2), -1)
	return ray.normalize()
}

// ray4 calculates the fourth ray-vector.
func ray4(FOVh, FOVv float64) *Vector {
	ray := newVector(-math.Tan(FOVv/2), math.Tan(FOVh/2), -1)
	return ray.normalize()
}

// rotateRays rotates the ray-vectors.
func rotateRays(ray1, ray2, ray3, ray4 *Vector, roll, pitch, yaw float64) []*Vector {
	sinAlpha, sinBeta, sinGamma := math.Sin(yaw), math.Sin(pitch), math.Sin(roll)
	cosAlpha, cosBeta, cosGamma := math.Cos(yaw), math.Cos(pitch), math.Cos(roll)
	m00, m01, m02 := cosAlpha*cosBeta, cosAlpha*sinBeta*sinGamma-sinAlpha*cosGamma, cosAlpha*sinBeta*cosGamma+sinAlpha*sinGamma
	m10, m11, m12 := sinAlpha*cosBeta, sinAlpha*sinBeta*sinGamma+cosAlpha*cosGamma, sinAlpha*sinBeta*cosGamma-cosAlpha*sinGamma
	m20, m21, m22 := -sinBeta, cosBeta*sinGamma, cosBeta*cosGamma

	rotationMatrix := [3][3]float64{
		{m00, m01, m02},
		{m10, m11, m12},
		{m20, m21, m22},
	}

	rotate := func(ray *Vector) *Vector {
		resX := rotationMatrix[0][0]*ray.x + rotationMatrix[0][1]*ray.y + rotationMatrix[0][2]*ray.z
		resY := rotationMatrix[1][0]*ray.x + rotationMatrix[1][1]*ray.y + rotationMatrix[1][2]*ray.z
		resZ := rotationMatrix[2][0]*ray.x + rotationMatrix[2][1]*ray.y + rotationMatrix[2][2]*ray.z
		return newVector(resX, resY, resZ)
	}

	return []*Vector{rotate(ray1), rotate(ray2), rotate(ray3), rotate(ray4)}
}

// getRayGroundIntersections finds intersections of rays with the ground.
func getRayGroundIntersections(rays []*Vector, origin *Vector) []*Vector {
	var intersections []*Vector
	for _, ray := range rays {
		intersections = append(intersections, findRayGroundIntersection(ray, origin))
	}
	return intersections
}

// findRayGroundIntersection finds a single ray's intersection with the ground.
func findRayGroundIntersection(ray, origin *Vector) *Vector {
	t := -(origin.z / ray.z)
	return newVector(origin.x+ray.x*t, origin.y+ray.y*t, origin.z+ray.z*t)
}

// converts degrees to radians.
func degreesToRadians(deg float64) float64 {
	return deg * math.Pi / 180
}

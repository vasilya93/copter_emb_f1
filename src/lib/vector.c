#include <math.h>
#include "vector.h"

float vector_len(const vector_t *vector)
{
  return sqrtf(vector->x * vector->x +
               vector->y * vector->y +
               vector->z * vector->z);
}

void vector_normalize(vector_t *vector)
{
  float len = vector_len(vector);
  vector->x = vector->x / len;
  vector->y = vector->y / len;
  vector->z = vector->z / len;
}

vector_t vector_multiply(const vector_t vector, float scalar)
{
  vector_t vector_result;
  vector_result.x = vector.x * scalar;
  vector_result.y = vector.y * scalar;
  vector_result.z = vector.z * scalar;
  return vector_result;
}

vector_t vector_divide(const vector_t vector, float scalar)
{
  vector_t vector_result;
  vector_result.x = vector.x / scalar;
  vector_result.y = vector.y / scalar;
  vector_result.z = vector.z / scalar;
  return vector_result;
}

vector_t vector_add_scalar(const vector_t vector, float scalar)
{
  vector_t vector_result;
  vector_result.x = vector.x + scalar;
  vector_result.y = vector.y + scalar;
  vector_result.z = vector.z + scalar;
  return vector_result;
}

vector_t vector_subtract_scalar(const vector_t vector, float scalar)
{
  vector_t vector_result;
  vector_result.x = vector.x - scalar;
  vector_result.y = vector.y - scalar;
  vector_result.z = vector.z - scalar;
  return vector_result;
}

vector_t vector_cross(const vector_t vector_a, const vector_t vector_b)
{
  vector_t vector_result;
  vector_result.x = vector_a.y * vector_b.z - vector_a.z * vector_b.y;
  vector_result.y = vector_a.z * vector_b.x - vector_a.x * vector_b.z;
  vector_result.z = vector_a.x * vector_b.y - vector_a.y * vector_b.x;
  return vector_result;
}

float vector_dot(const vector_t vector_a, const vector_t vector_b)
{
  return vector_a.x * vector_b.x +
         vector_a.y * vector_b.y +
         vector_a.z * vector_b.z;
}

vector_t vector_add(const vector_t vector_a, const vector_t vector_b)
{
  vector_t vector_result;
  vector_result.x = vector_a.x + vector_b.x;
  vector_result.y = vector_a.y + vector_b.y;
  vector_result.z = vector_a.z + vector_b.z;
  return vector_result;
}

vector_t vector_subtract(const vector_t vector_a, const vector_t vector_b)
{
  vector_t vector_result;
  vector_result.x = vector_a.x - vector_b.x;
  vector_result.y = vector_a.y - vector_b.y;
  vector_result.z = vector_a.z - vector_b.z;
  return vector_result;
}

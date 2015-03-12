#ifndef VECTOR_H
#define VECTOR_H

typedef struct vector_t {
        float x;
        float y;
        float z;
}vector_t;

float vector_len(const vector_t *vector);
void vector_normalize(vector_t *vector);
vector_t vector_multiply(const vector_t vector, float scalar);
vector_t vector_divide(const vector_t vector, float scalar);
vector_t vector_add_scalar(const vector_t vector, float scalar);
vector_t vector_subtract_scalar(const vector_t vector, float scalar);
vector_t vector_cross(const vector_t vector_a, const vector_t vector_b);
float vector_dot(const vector_t vector_a, const vector_t vector_b);
vector_t vector_add(const vector_t vector_a, const vector_t vector_b);
vector_t vector_subtract(const vector_t vector_a, const vector_t vector_b);

#endif /* VECTOR_H */
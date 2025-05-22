namespace percepto::core
{

/**
 * @brief Intersectable (CRTP Interface)
 *
 * A compile-time polymorphic base class for geometric objects that can be
 * intersected by a ray. This interface uses the Curiously Recurring Template
 * Pattern (CRTP) to allow derived classes (e.g., Sphere, Triangle, Plane) to
 * implement their own `intercept` method without incurring runtime overhead.
 *
 * CRTP enables compile-time polymorphism, which allows:
 * - Elimination of virtual tables (vtables)
 * - Full compile-time dispatch
 * - Aggressive inlining and compiler optimization
 */

template <typename Derived>
class Intersectable
{
 public:
  // Dispatches the call to the derived class’s implementation of `intercept`
  // using CRTP. Returns true if the ray hits the object, and writes the hit
  // distance to `t`.
  [[nodiscard]]
  bool intercept(const Ray& ray, double& t_hit) const
  {
    return static_cast<const Derived*>(this)->intercept(ray, t_hit);
  }
};

}  // namespace percepto::core

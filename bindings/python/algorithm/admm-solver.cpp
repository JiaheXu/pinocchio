//
// Copyright (c) 2024 INRIA
//

#include "pinocchio/bindings/python/fwd.hpp"

#include "pinocchio/algorithm/admm-solver.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/delassus-operator-plain.hpp"
#include "pinocchio/algorithm/delassus-operator-sparse.hpp"

#include "pinocchio/bindings/python/algorithm/contact-solver-base.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include <eigenpy/eigen-from-python.hpp>
#include <eigenpy/eigen-to-python.hpp>

namespace pinocchio
{
namespace python
{
  namespace bp = boost::python;

  typedef ADMMContactSolverTpl<context::Scalar> Solver;
  typedef context::Scalar Scalar;
  typedef context::VectorXs VectorXs;
  typedef const Eigen::Ref<const VectorXs> ConstRefVectorXs;
  typedef ContactCholeskyDecompositionTpl<context::Scalar,context::Options> ContactCholeskyDecomposition;

#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE

  template<typename DelassusDerived>
  static bool solve_wrapper(Solver & solver,
                            DelassusDerived & delassus,
                            const context::VectorXs & g,
                            const context::CoulombFrictionConeVector & cones,
                            const context::VectorXs & R,
                            const boost::optional<ConstRefVectorXs> primal_solution = boost::none,
                            const boost::optional<ConstRefVectorXs> dual_solution = boost::none,
                            bool compute_largest_eigen_values = true)
  {
    return solver.solve(delassus,g,cones,R,primal_solution,dual_solution,compute_largest_eigen_values);
  }

  template<typename DelassusDerived>
  static bool solve_wrapper2(Solver & solver,
                             DelassusDerived & delassus,
                             const context::VectorXs & g,
                             const context::CoulombFrictionConeVector & cones,
                             Eigen::Ref<context::VectorXs> x)
  {
    return solver.solve(delassus,g,cones,x);
  }
#endif

  void exposeADMMContactSolver()
  {
#ifdef PINOCCHIO_PYTHON_PLAIN_SCALAR_TYPE
    bp::class_<Solver>("ADMMContactSolver",
                       "Alternating Direction Method of Multi-pliers solver for contact dynamics.",
                       bp::init<int, Scalar, Scalar, Scalar, Scalar, Scalar, int>((bp::arg("self"),
                                                                                   bp::arg("problem_dim"),
                                                                                   bp::arg("mu_prox") = Scalar(1e-6),
                                                                                   bp::arg("tau") = Scalar(0.5),
                                                                                   bp::arg("rho_power") = Scalar(0.2),
                                                                                   bp::arg("rho_power_factor") = Scalar(0.05),
                                                                                   bp::arg("ratio_primal_dual") = Scalar(10),
                                                                                   bp::arg("max_it_largest_eigen_value_solver") = 10),
                                                                     "Default constructor."))
    .def(ContactSolverBasePythonVisitor<Solver>())

    .def("solve",solve_wrapper<ContactCholeskyDecomposition::DelassusCholeskyExpression>,
         (bp::args("self","delassus","g","cones","R"),
          bp::arg("primal_solution") = boost::none,
          bp::arg("dual_solution") = boost::none,
          bp::arg("compute_largest_eigen_values") = true),
         "Solve the constrained conic problem, starting from the optional initial guess.")
    .def("solve",solve_wrapper<context::DelassusOperatorDense>,
         (bp::args("self","delassus","g","cones","R"),
          bp::arg("primal_solution") = boost::none,
          bp::arg("dual_solution") = boost::none,
          bp::arg("compute_largest_eigen_values") = true),
         "Solve the constrained conic problem, starting from the optional initial guess.")
    .def("solve",solve_wrapper<context::DelassusOperatorSparse>,
         (bp::args("self","delassus","g","cones","R"),
          bp::arg("primal_solution") = boost::none,
          bp::arg("dual_solution") = boost::none,
          bp::arg("compute_largest_eigen_values") = true),
         "Solve the constrained conic problem, starting from the optional initial guess.")

    .def("setRho",&Solver::setRho,bp::args("self","rho"),
         "Set the ADMM penalty value.")
    .def("getRho",&Solver::getRho,bp::arg("self"),
         "Get the ADMM penalty value.")

    .def("setRhoPower",&Solver::setRhoPower,bp::args("self","rho_power"),
         "Set the power associated to the problem conditionning.")
    .def("getRhoPower",&Solver::getRhoPower,bp::arg("self"),
         "Get the power associated to the problem conditionning.")

    .def("setRhoPowerFactor",&Solver::setRhoPowerFactor,bp::args("self","rho_power_factor"),
         "Set the power factor associated to the problem conditionning.")
    .def("getRhoPowerFactor",&Solver::getRhoPowerFactor,bp::arg("self"),
         "Get the power factor associated to the problem conditionning.")

    .def("setTau",&Solver::setTau,bp::args("self","tau"),
         "Set the tau linear scaling factor.")
    .def("getTau",&Solver::getTau,bp::arg("self"),
         "Get the tau linear scaling factor.")

    .def("setProximalValue",&Solver::setProximalValue,bp::args("self","mu"),
         "Set the proximal value.")
    .def("getProximalValue",&Solver::getProximalValue,bp::arg("self"),
         "Get the proximal value.")

    .def("setRatioPrimalDual",&Solver::setRatioPrimalDual,bp::args("self","ratio_primal_dual"),
         "Set the primal/dual ratio.")
    .def("getRatioPrimalDual",&Solver::getRatioPrimalDual,bp::arg("self"),
         "Get the primal/dual ratio.")
    
    .def("getPrimalSolution",&Solver::getPrimalSolution,bp::arg("self"),
         "Returns the primal solution of the problem.",bp::return_internal_reference<>())

    .def("getDualSolution",&Solver::getDualSolution,bp::arg("self"),
         "Returns the dual solution of the problem.",bp::return_internal_reference<>())

    .def("getCholeskyUpdateCount",&Solver::getCholeskyUpdateCount,bp::arg("self"),
         "Returns the number of updates of the Cholesky factorization due to rho updates.")

    .def("computeRho",&Solver::computeRho,bp::args("L","m","rho_power"),
         "Compute the penalty ADMM value from the current largest and lowest Eigen values and the scaling spectral factor.")
    .staticmethod("computeRho")
    .def("computeRhoPower",&Solver::computeRhoPower,bp::args("L","m","rho"),
         "Compute the  scaling spectral factor of the ADMM penalty term from the current largest and lowest Eigen values and the ADMM penalty term.")
    .staticmethod("computeRhoPower")
    ;
#endif
  }


} // namespace python
} // namespace pinocchio

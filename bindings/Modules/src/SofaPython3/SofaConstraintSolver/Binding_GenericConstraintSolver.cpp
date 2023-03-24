/******************************************************************************
*                              SofaPython3 plugin                             *
*                  (c) 2021 CNRS, University of Lille, INRIA                  *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <pybind11/pybind11.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <SofaPython3/Sofa/Core/Binding_ForceField_doc.h>
#include <SofaPython3/SofaConstraintSolver/Binding_GenericConstraintSolver.h>
#include <SofaPython3/SofaConstraintSolver/Binding_GenericConstraintSolver_doc.h>

namespace py { using namespace pybind11; }
namespace sofapython3
{
void moduleAddGenericConstraintSolver(pybind11::module &m)
{
    const std::string pyclass_name = sofa::component::constraint::lagrangian::solver::GenericConstraintSolver::GetClass()->className;
    py::class_<sofa::component::constraint::lagrangian::solver::GenericConstraintSolver,
               sofa::core::objectmodel::BaseObject,
               GenericConstraintSolver_trampoline,
               py_shared_ptr<sofa::component::constraint::lagrangian::solver::GenericConstraintSolver>> f
    (m, pyclass_name.c_str(), py::dynamic_attr(), py::multiple_inheritance(), sofapython3::doc::constraintsolver::genericConstraintSolverClass);

}
}

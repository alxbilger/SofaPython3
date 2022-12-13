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

#include <sofa/core/behavior/SingleStateAccessor.h>
#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_SingleStateAccessor.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>

namespace py { using namespace pybind11; }

namespace sofa::core::behavior
{
extern template class SOFA_CORE_API SingleStateAccessor< sofa::defaulttype::Vec3Types >;
extern template class SOFA_CORE_API SingleStateAccessor< sofa::defaulttype::Vec2Types >;
extern template class SOFA_CORE_API SingleStateAccessor< sofa::defaulttype::Vec1Types >;
extern template class SOFA_CORE_API SingleStateAccessor< sofa::defaulttype::Vec6Types >;
extern template class SOFA_CORE_API SingleStateAccessor< sofa::defaulttype::Rigid3Types >;
extern template class SOFA_CORE_API SingleStateAccessor< sofa::defaulttype::Rigid2Types >;
}

namespace sofapython3 {

template<class TDOFType>
void declareSingleStateAccessor(py::module &m)
{
    const std::string pyclass_name = std::string("SingleStateAccessor") + TDOFType::Name();
    py::class_<sofa::core::behavior::SingleStateAccessor<TDOFType>,
               sofa::core::objectmodel::BaseObject, py_shared_ptr<sofa::core::behavior::SingleStateAccessor<TDOFType>>> f(m, pyclass_name.c_str());

    f.def("get_mstate", [](sofa::core::behavior::SingleStateAccessor<TDOFType>& self) -> py::object
    {
        sofa::core::behavior::MechanicalState<TDOFType>* mstate = self.getMState();
        if (mstate)
        {
            return py::cast(static_cast<sofa::core::objectmodel::BaseObject*>(mstate));
        }
        return py::none();
    });

    PythonFactory::registerType<sofa::core::behavior::SingleStateAccessor<TDOFType>>([](sofa::core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<sofa::core::behavior::SingleStateAccessor<TDOFType>*>(object));
    });
}

void moduleAddSingleStateAccessor(py::module& m)
{
    declareSingleStateAccessor<sofa::defaulttype::Vec3dTypes>(m);
    declareSingleStateAccessor<sofa::defaulttype::Vec2dTypes>(m);
    declareSingleStateAccessor<sofa::defaulttype::Vec1dTypes>(m);
    declareSingleStateAccessor<sofa::defaulttype::Vec6dTypes>(m);
    declareSingleStateAccessor<sofa::defaulttype::Rigid3dTypes>(m);
    declareSingleStateAccessor<sofa::defaulttype::Rigid2dTypes>(m);
}

} // namespace sofapython3

#include <SofaPython3/SofaDeformable/Binding_JaxFEMForceField.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <SofaPython3/SofaDeformable/JaxFEMForceField.inl>
#include <sofa/fem/FiniteElement[all].h>

namespace sofapython3
{

namespace py { using namespace pybind11; }

template <class TDataTypes, class TElementType>
void declareJaxFEMForceField(py::module &m)
{
    const auto elementType = sofa::geometry::elementTypeToString(TElementType::Element_type);
    const std::string pyclass_name = elementType + std::string("JaxFEMForceField") + TDataTypes::Name();

    using Class = JaxFEMForceField<TDataTypes, TElementType>;
    using Base = sofa::core::behavior::ForceField<TDataTypes>;

    py::class_<Class, Base,
               py_shared_ptr<Class>> f(m, pyclass_name.c_str(), py::dynamic_attr());

    f.def("energy", [](Class & /*self*/, py::object /*F*/) {
        return py::none(); // To be overridden in Python
    });

    PythonFactory::registerType<JaxFEMForceField<TDataTypes, TElementType>>([](sofa::core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<JaxFEMForceField<TDataTypes, TElementType>*>(object));
    });
}

void moduleAddJaxFEMForceField(py::module &m)
{
    // declareJaxFEMForceField<sofa::defaulttype::Vec1Types, sofa::geometry::Edge>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec2Types, sofa::geometry::Edge>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec3Types, sofa::geometry::Edge>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec2Types, sofa::geometry::Triangle>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec3Types, sofa::geometry::Triangle>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec2Types, sofa::geometry::Quad>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec3Types, sofa::geometry::Quad>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec3Types, sofa::geometry::Tetrahedron>(m);
    declareJaxFEMForceField<sofa::defaulttype::Vec3Types, sofa::geometry::Hexahedron>(m);
}

}

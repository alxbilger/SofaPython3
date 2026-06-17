project(Bindings.Modules.SofaDeformable)

set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_JaxFEMForceField.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_LinearSpring.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_SpringForceField.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/JaxFEMForceField.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Module_SofaDeformable.cpp
)

set(HEADER_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_JaxFEMForceField.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_LinearSpring.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_LinearSpring_doc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_SpringForceField.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Binding_SpringForceField_doc.h
    ${CMAKE_CURRENT_SOURCE_DIR}/JaxFEMForceField.h
    ${CMAKE_CURRENT_SOURCE_DIR}/JaxFEMForceField.inl
)

if (NOT TARGET SofaPython3::Plugin)
    find_package(SofaPython3 REQUIRED COMPONENTS Plugin Bindings.Sofa)
endif()

sofa_find_package(Sofa.Component.SolidMechanics.Spring REQUIRED)

SP3_add_python_module(
    TARGET       ${PROJECT_NAME}
    PACKAGE      Bindings.Modules
    MODULE       SofaDeformable
    DESTINATION  Sofa
    SOURCES      ${SOURCE_FILES}
    HEADERS      ${HEADER_FILES}
    DEPENDS      Sofa.Component.SolidMechanics.Spring SofaPython3::Plugin SofaPython3::Bindings.Sofa.Core

)

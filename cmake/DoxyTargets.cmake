add_custom_target(doxy-doc
  COMMAND ${Flowlessly_ROOT_DIR}/contrib/generate-documentation.sh ${Flowlessly_ROOT_DIR}
  COMMENT "Generating Doxy documentation...")

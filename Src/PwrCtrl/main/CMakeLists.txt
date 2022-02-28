set(COMPONENT_SRCS "app_main.c" "app_wifi.c" "app_httpd.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_EMBED_FILES
        "www/index_ov2640.html.gz"
        "www/monitor.html.gz")

register_component()

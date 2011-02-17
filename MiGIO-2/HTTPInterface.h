#ifndef HTTP_INTERFACE_H
#define HTTP_INTERFACE_H


/** Fetchs a URL and optional saves to a file.*/
void http_fetch( const char *url, const char *filename="");
/** Fetchs a URL and returns the data in a freshly malloc'd cstring */
char *http_strdup( const char *url );
void http_interface_init();
void http_interface_destroy();

#endif

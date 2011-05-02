#include "HTTPInterface.h"
#include <string>
#include <iostream>
#include <fstream>
#include <cstring>
#include "curl/curl.h"  


using namespace std;

//static char errorBuffer[CURL_ERROR_SIZE];
static string imageBuffer; 
static int result;

// Writer Callback
static int writer(char *data, size_t size, size_t nmemb, std::string *buffer)
{
	int result = 0;
	// Is there anything in the buffer?
	if (buffer != NULL)
	{
		buffer->append(data, size * nmemb);
		result = size * nmemb;
	}
	return result;
}  


void http_interface_init() {
}

void http_interface_destroy() {
}

void http_fetch( const char *url, const char *filename ) {
        CURL *curl = curl_easy_init();  
		if (!filename) filename="/dev/null";
        int do_file = filename && strlen(filename) >= 1;
        
        FILE * fptr;
        if( do_file ) {
            fptr = fopen( filename, "wb" );
        }

	curl_easy_setopt(curl, CURLOPT_USERAGENT, "Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.7.5) Gecko/20041107 Firefox/1.0");
	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_HEADER, 0);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
        if( do_file ) {
            /* send all data to this function  */ 
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, 
                             fwrite);
            
            /* we pass our 'chunk' struct to the callback function */ 
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, 
                             (void *)fptr);
        }

	// Attempt to retrieve the remote page
	result = curl_easy_perform(curl);


        if( do_file ) {
            //fflush( fptr );
            fclose( fptr );
        }
	curl_easy_cleanup(curl); 
}

char *http_strdup( const char *url ) {
        CURL *curl = curl_easy_init();  
        
        std::string buffer;
	curl_easy_setopt(curl, CURLOPT_USERAGENT, "Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.7.5) Gecko/20041107 Firefox/1.0");
	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_HEADER, 0);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
        /* send all data to this function  */ 
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, 
                         writer);
            
        /* we pass our 'chunk' struct to the callback function */ 
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, 
                         (void *)&buffer);

	// Attempt to retrieve the remote page
	result = curl_easy_perform(curl);

	curl_easy_cleanup(curl);
        const char *cstr = buffer.c_str();
        return strdup(cstr);

}

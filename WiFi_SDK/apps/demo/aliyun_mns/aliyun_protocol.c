/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "aliyun_protocol.h"

#include "xml.h"
#include "wiced.h"
#include "wiced_time.h"
#include "http_client.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define WICED_VERIFY_TRUE(x)                do { if ( !(x) ) {return WICED_ERROR;}  } while(0)
#define B64LENGTH(x)                        ((((4*strlen(x)/3)+3)&~3)+1)

/******************************************************
 *                    Constants
 ******************************************************/

const char *month_s[]={
        "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"
};

const char *day_of_week_s[]={
        "Sun","Mon","Tue","Wed","Thu","Fri","Sat"
};

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static const char* http_methods[] =
{
        [HTTP_OPTIONS]  =  HTTP_METHOD_OPTIONS,
        [HTTP_GET    ]  =  HTTP_METHOD_GET,
        [HTTP_HEAD   ]  =  HTTP_METHOD_HEAD,
        [HTTP_POST   ]  =  HTTP_METHOD_POST,
        [HTTP_PUT    ]  =  HTTP_METHOD_PUT,
        [HTTP_DELETE ]  =  HTTP_METHOD_DELETE,
        [HTTP_TRACE  ]  =  HTTP_METHOD_TRACE,
        [HTTP_CONNECT]  =  HTTP_METHOD_CONNECT,
};

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t hard_clear_buffers(wiced_aliyun_buffers_t *buffers)
{
    if( ! (buffers->date_string && buffers->resource && buffers->sig_plain && buffers->signature && buffers->xmns && buffers->authorization && buffers->content_length) )
    {
        return WICED_ERROR;
    }
    memset(buffers->date_string, 0, DATE_BUFFER_SIZE);
    memset(buffers->resource, 0, RESOURCE_BUFFER_SIZE);
    memset(buffers->sig_plain, 0, SIGNATURE_PLAINTEXT_BUFFER_SIZE);
    memset(buffers->signature, 0, SIGNATURE_BUFFER_SIZE);
    memset(buffers->xmns, 0, XMNS_BUFFER_SIZE);
    memset(buffers->authorization, 0, AUTHORIZATION_BUFFER_SIZE);
    memset(buffers->content_length, 0, CONTENT_LENGTH_BUFFER_SIZE);

    return WICED_SUCCESS;
}

wiced_result_t reset( wiced_aliyun_buffers_t *buffers, wiced_xml_t *xml)
{
    WICED_VERIFY( hard_clear_buffers( buffers ) );
    WICED_VERIFY( wiced_xml_reset(xml) );
    return WICED_SUCCESS;
}

wiced_result_t wiced_aliyun_init( wiced_aliyun_t *aliyun, wiced_aliyun_queue_t *aliyun_queue,  wiced_aliyun_buffers_t *aliyun_buffers, wiced_xml_t *xml, http_header_field_t *http_header, http_client_t *http_client )
{
    if( ! (aliyun->wiced_aliyun_queue && aliyun->wiced_aliyun_buffers && aliyun->wiced_xml && aliyun->http_header && aliyun->http_client) )
    {
        return WICED_ERROR;
    }
    aliyun->wiced_aliyun_queue = aliyun_queue;
    aliyun->wiced_aliyun_buffers = aliyun_buffers;
    aliyun->wiced_xml = xml;
    aliyun->http_header = http_header;
    aliyun->http_client = http_client;

    WICED_VERIFY( reset( aliyun->wiced_aliyun_buffers, aliyun->wiced_xml ) );

    return WICED_SUCCESS;
}

void dump_bytes(const uint8_t* bptr, uint32_t len)
{
    int i = 0;

    for (i = 0; i < len; )
    {
        if ((i & 0x0f) == 0)
        {
            WPRINT_APP_INFO( ( "\n" ) );
        }
        else if ((i & 0x07) == 0)
        {
            WPRINT_APP_INFO( (" ") );
        }
        WPRINT_APP_INFO( ( "%02x ", bptr[i++] ) );
    }
    WPRINT_APP_INFO( ( "\n" ) );
}

char upper_to_lower(char input)
{
    return ((input >= 'A') && (input <= 'Z')) ? (input + ('a'-'A')) : (input);
}

http_method_t api_call_to_method(wiced_aliyun_queue_function_t apicall)
{
    switch(apicall){
    case CREATEQUEUE:
    case SETQUEUEATTRIBUTES:
    case CHANGEMESSAGEVISIBILITY:
        return HTTP_PUT;
        break;
    case GETQUEUEATTRIBUTES:
    case LISTQUEUE:
    case RECEIVEMESSAGE:
    case BATCHRECEIVEMESSAGE:
    case PEEKMESSAGE:
    case BATCHPEEKMESSAGE:
        return HTTP_GET;
        break;
    case DELETEQUEUE:
    case DELETEMESSAGE:
    case BATCHDELETEMESSAGE:
        return HTTP_DELETE;
        break;
    case SENDMESSAGE:
    case BATCHSENDMESSAGE:
        return HTTP_POST;
        break;
    default:
        return HTTP_UNKNOWN;
    }
}

int day_of_week(int year, int month, int day)
{
    static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    year -= month < 3;
    return (year + year/4 -year/100 + year/400 + t[month-1] + day) % 7;
}

wiced_result_t create_date_string(wiced_iso8601_time_t timestamp, char* buffer, int buffer_size)
{
    int year,month,day,hour,minute,second;
    long subsecond;
    int result = sscanf( (char*)&timestamp, "%d-%d-%dT%d:%d:%d.%ld",&year,&month,&day,&hour,&minute,&second,&subsecond);
    WICED_VERIFY_TRUE( result == 7 );
    result = snprintf(buffer,buffer_size,"%s, %02d %s %04d %02d:%02d:%02d GMT",day_of_week_s[day_of_week(year,month,day)],day,month_s[month-1],year,hour,minute,second);
    WICED_VERIFY_TRUE( result != -1 );
    return WICED_SUCCESS;
}

int32_t create_signature_string_plaintext(http_method_t method, char* contentmd5, char* contenttype, char* date, char* canonicalizedMNSHeaders, char* canonicalizedResource, char* sig_string, int buffer_size)
{
    int returnsize;
    if( (contentmd5 == NULL) && (contenttype == NULL) ){
        returnsize = snprintf(sig_string,buffer_size,"%s\n\n\n%s\n%s\n%s",http_methods[method],date,canonicalizedMNSHeaders,canonicalizedResource);
    }
    else if( contentmd5 == NULL ){
        returnsize = snprintf(sig_string,buffer_size,"%s\n\n%s\n%s\n%s\n%s",http_methods[method],contenttype,date,canonicalizedMNSHeaders,canonicalizedResource);
    }
    else if( contenttype == NULL ){
        returnsize = snprintf(sig_string,buffer_size,"%s\n%s\n\n%s\n%s\n%s",http_methods[method],contentmd5,date,canonicalizedMNSHeaders,canonicalizedResource);
    }
    else{
        returnsize = snprintf(sig_string,buffer_size,"%s\n%s\n%s\n%s\n%s\n%s",http_methods[method],contentmd5,contenttype,date,canonicalizedMNSHeaders,canonicalizedResource);
    }
    wiced_assert( "Create signature string plaintext error\n", returnsize != -1 );
    return returnsize;
}

void sign_signature_string( char* sig_string_plain, int32_t sig_string_len, char* secret_key, int32_t secret_key_size, char* signature_buffer, int32_t signature_buffer_size)
{
    //length of crypto result fixed at 20
    unsigned char cryptoresult[20];
    sha1_hmac((unsigned char*)secret_key,secret_key_size,(unsigned char*)sig_string_plain,sig_string_len,cryptoresult);
    int result = base64_encode( cryptoresult, 20, (unsigned char*)signature_buffer, signature_buffer_size, BASE64_STANDARD );
    wiced_assert( "Sign signature string error\n", result > 0 );
}

wiced_result_t create_canonicalized_xmns_headers(char *xmns_marker, char *xmns_ret_number, char *xmns_prefix, char *xmns_buffer, int xmns_buffer_size)
{
    int offset = 0, snprintfresult;

    snprintfresult = snprintf(xmns_buffer, xmns_buffer_size, "%s%s\n", HTTP_HEADER_XMNS_VERSION, ALIYUN_XMNS_VERSION);
    WICED_VERIFY_TRUE( snprintfresult != -1 );
    offset += snprintfresult;
    if( xmns_marker != NULL){
        snprintfresult = snprintf(xmns_buffer+offset, xmns_buffer_size-offset, "%s%s\n", HTTP_HEADER_XMNS_MARKER, xmns_marker);
        WICED_VERIFY_TRUE( snprintfresult != -1 );
        offset += snprintfresult;
    }
    if( xmns_ret_number != NULL){
        snprintfresult = snprintf(xmns_buffer+offset, xmns_buffer_size-offset, "%s%s\n", HTTP_HEADER_XMNS_RET_NUMBER, xmns_ret_number);
        WICED_VERIFY_TRUE( snprintfresult != -1 );
        offset += snprintfresult;
    }
    if( xmns_prefix != NULL){
        snprintfresult = snprintf(xmns_buffer+offset, xmns_buffer_size-offset, "%s%s\n", HTTP_HEADER_XMNS_PREFIX, xmns_prefix);
        WICED_VERIFY_TRUE( snprintfresult != -1 );
        offset += snprintfresult;
    }
    xmns_buffer[offset-1]='\0';//remove last \n
    return WICED_SUCCESS; //if all asserts pass, success
}

wiced_result_t create_resource( wiced_aliyun_queue_function_t apicall, char *queuename, char* val1, char* val2, char* buffer, int buffer_size)
{
    int snprintfresult = -1;
    switch(apicall){
    case LISTQUEUE:
        snprintfresult = snprintf(buffer,buffer_size, "/queues");
        break;
    case SETQUEUEATTRIBUTES:
        snprintfresult = snprintf(buffer,buffer_size,"/queues/%s?metaoverride=true",queuename);
        break;
    case CREATEQUEUE:
    case GETQUEUEATTRIBUTES:
    case DELETEQUEUE:
        snprintfresult = snprintf(buffer,buffer_size, "/queues/%s",queuename);
        break;
    case BATCHRECEIVEMESSAGE:
        if(val2==NULL){
            snprintfresult = snprintf(buffer,buffer_size, "/queues/%s/messages?numOfMessages=%s",queuename,val1);
        }
        else{
            snprintfresult = snprintf(buffer,buffer_size, "/queues/%s/messages?numOfMessages=%s&waitseconds=%s",queuename,val1,val2);
        }
        break;
    case BATCHPEEKMESSAGE:
        snprintfresult = snprintf(buffer,buffer_size, "/queues/%s/messages?peekonly=true&numOfMessages=%s",queuename,val1);
        break;
    case CHANGEMESSAGEVISIBILITY:
        snprintfresult = snprintf(buffer, buffer_size,"/queues/%s/messages?messageHandle=%s&visibilityTimeout=%s",queuename,val1,val2);
        break;
    case RECEIVEMESSAGE:
        if(val1==NULL){
            snprintfresult = snprintf(buffer,buffer_size, "/queues/%s/messages",queuename);
        }
        else{
            snprintfresult = snprintf(buffer,buffer_size, "/queues/%s/messages?waitseconds=%s",queuename,val1);
        }
        break;
    case DELETEMESSAGE:
        snprintfresult = snprintf(buffer,buffer_size, "/queues/%s/messages?ReceiptHandle=%s",queuename,val1);
        break;
    case PEEKMESSAGE:
        snprintfresult = snprintf(buffer, buffer_size, "/queues/%s/messages?peekonly=true",queuename);
        break;
    case SENDMESSAGE:
    case BATCHSENDMESSAGE:
    case BATCHDELETEMESSAGE:
        snprintfresult = snprintf(buffer,buffer_size, "/queues/%s/messages",queuename);
    }
    WICED_VERIFY_TRUE( snprintfresult != -1 );
    return WICED_SUCCESS;
}

void fill_header( http_header_field_t *header, char *field, char *value)
{
    header->field = field;
    header->field_length = (uint16_t)strlen(field);
    header->value = value;
    header->value_length = (uint16_t)strlen(value);
}

uint8_t create_request(wiced_aliyun_queue_function_t apicall, wiced_aliyun_queue_t *queue_config, wiced_aliyun_buffers_t *buffers, wiced_xml_t *xml, http_header_field_t *headers, char *resource_val_1, char *resource_val_2, char *resource_val_3)
{
    wiced_iso8601_time_t timestamp;
    wiced_time_get_iso8601_time(&timestamp);
    WICED_VERIFY( create_date_string(timestamp, buffers->date_string, DATE_BUFFER_SIZE) );
    WICED_VERIFY( create_resource(apicall,queue_config->queue_name,resource_val_1,resource_val_2,buffers->resource,RESOURCE_BUFFER_SIZE) );
    if(apicall == LISTQUEUE)
    {
        WICED_VERIFY( create_canonicalized_xmns_headers(resource_val_1, resource_val_2, resource_val_3, buffers->xmns, XMNS_BUFFER_SIZE) );
    }
    else
    {
        WICED_VERIFY( create_canonicalized_xmns_headers(NULL, NULL, NULL, buffers->xmns, XMNS_BUFFER_SIZE) );
    }

    int sig_len;
#ifdef INCLUDE_MD5 //broken
    if(xml->bytes_used != 0)
    {
        char md5out[17];
        memset(md5out,0,sizeof(md5out));
        md5((unsigned char*)xml->buffer, strlen(xml->buffer),(unsigned char *)md5out);
        char md5out_encoded[32];
        base64_encode( md5out, 17, (unsigned char*)md5out_encoded, 32, BASE64_STANDARD );
        sig_len = create_signature_string_plaintext( api_call_to_method(apicall),
                md5out_encoded,
                ALIYUN_CONTENT_TYPE,
                buffers->date_string,
                buffers->xmns,
                buffers->resource,
                buffers->sig_plain,
                SIGNATURE_PLAINTEXT_BUFFER_SIZE);
    }
    else
    {
        sig_len = create_signature_string_plaintext( api_call_to_method(apicall),
                NULL,
                NULL,
                buffers->date_string,
                buffers->xmns,
                buffers->resource,
                buffers->sig_plain,
                SIGNATURE_PLAINTEXT_BUFFER_SIZE);
    }
#else
    if( xml->bytes_used != 0)
    {
        sig_len = create_signature_string_plaintext( api_call_to_method(apicall),
                NULL,
                ALIYUN_CONTENT_TYPE,
                buffers->date_string,
                buffers->xmns,
                buffers->resource,
                buffers->sig_plain,
                SIGNATURE_PLAINTEXT_BUFFER_SIZE);
    }
    else
    {
        sig_len = create_signature_string_plaintext( api_call_to_method(apicall),
                NULL,
                NULL,
                buffers->date_string,
                buffers->xmns,
                buffers->resource,
                buffers->sig_plain,
                SIGNATURE_PLAINTEXT_BUFFER_SIZE);
    }
#endif

    sign_signature_string( buffers->sig_plain, sig_len, queue_config->secret_key, strlen(queue_config->secret_key), buffers->signature, SIGNATURE_BUFFER_SIZE);

    int snprintfreturn = snprintf( buffers->authorization, AUTHORIZATION_BUFFER_SIZE, "MNS %s:%s", ALIYUN_ACCESS_KEY, buffers->signature);
    wiced_assert( "Create authorization string error\n", snprintfreturn != -1 );

    uint8_t num_headers = 0;

    fill_header( &headers[num_headers], HTTP_HEADER_HOST, ALIYUN_HOST_NO_HTTP );
    num_headers++;
    fill_header( &headers[num_headers], HTTP_HEADER_DATE, buffers->date_string );
    num_headers++;
    if( xml->bytes_used != 0 )
    {
        fill_header( &headers[num_headers], HTTP_HEADER_CONTENT_TYPE, ALIYUN_CONTENT_TYPE );
        num_headers++;
        snprintf(buffers->content_length,CONTENT_LENGTH_BUFFER_SIZE,"%d",(int)xml->bytes_used);
        WPRINT_APP_DEBUG( ("content length: %s\n",length_buf));
        fill_header( &headers[num_headers], HTTP_HEADER_CONTENT_LENGTH, buffers->content_length );
        num_headers++;
    }
    fill_header( &headers[num_headers], HTTP_HEADER_XMNS_VERSION, ALIYUN_XMNS_VERSION );
    num_headers++;
    fill_header( &headers[num_headers], HTTP_HEADER_AUTHORIZATION, buffers->authorization );
    num_headers++;
    if( apicall == LISTQUEUE )
    {
        if( resource_val_1 != NULL )
        {
            fill_header( &headers[num_headers], HTTP_HEADER_XMNS_MARKER, resource_val_1 );
            num_headers++;
        }
        if( resource_val_2 != NULL )
        {
            fill_header( &headers[num_headers], HTTP_HEADER_XMNS_RET_NUMBER, resource_val_2 );
            num_headers++;
        }
        if( resource_val_3 != NULL )
        {
            fill_header( &headers[num_headers], HTTP_HEADER_XMNS_PREFIX, resource_val_3 );
            num_headers++;
        }
    }

    if( xml->bytes_used != 0)
    {
        fill_header( &headers[num_headers], HTTP_CLRF, xml->buffer );
        num_headers++;
    }
    return num_headers;
}

void print_buffers(wiced_aliyun_buffers_t *buffers)
{
    WPRINT_APP_INFO(("////===BUFFERS===\\\\\\\\\n"));
    WPRINT_APP_INFO(("Date string:\n%s\n\n",buffers->date_string));
    WPRINT_APP_INFO(("Resource:\n%s\n\n",buffers->resource));
    WPRINT_APP_INFO(("Sig Plain:\n%s\n\n",buffers->sig_plain));
    WPRINT_APP_INFO(("Signature:\n%s\n\n",buffers->signature));
    WPRINT_APP_INFO(("XMNS:\n%s\n\n",buffers->xmns));
    WPRINT_APP_INFO(("Authorization:\n%s\n\n",buffers->authorization));
    WPRINT_APP_INFO(("Content Length:\n%s\n\n",buffers->content_length));
}

wiced_result_t wiced_encode_message( char* message, char* storage, uint16_t storage_length)
{
    int result = base64_encode((unsigned char*)message,strlen(message),(unsigned char*)storage,storage_length,BASE64_STANDARD);
    WICED_VERIFY_TRUE( result != -1 );
    return WICED_SUCCESS;
}

wiced_result_t send_request( wiced_aliyun_queue_function_t queue_function, http_request_t *http_request, http_client_t *http_client, http_header_field_t *http_header, uint8_t num_headers, wiced_aliyun_buffers_t *buffers )
{
    WPRINT_APP_INFO (( "send_request\r\n"));
    WICED_VERIFY( http_request_init( http_request, http_client, api_call_to_method(queue_function), buffers->resource, DEFAULT_HTTP_VERSION ) );
    WICED_VERIFY( http_request_write_header( http_request, http_header, num_headers ) );
    WICED_VERIFY( http_request_write_end_header( http_request ) );
#if ENABLE_PRINT_BUFFERS
    WPRINT_APP_INFO( ("////===Request Dump===\\\\\\\\\n") );
    dump_bytes(http_request->stream.tx_packet_data-http_request->stream.tx_packet_data_length,http_request->stream.tx_packet_data_length );
    print_buffers(buffers);
#endif
    WICED_VERIFY( http_request_flush( http_request ) );
    return WICED_SUCCESS;
}

wiced_result_t wiced_aliyun_execute_function( wiced_aliyun_t *aliyun, http_request_t* request, wiced_aliyun_queue_function_t queue_function, char *option1, char *option2, char *option3 )
{
    uint8_t num_headers = create_request(queue_function, aliyun->wiced_aliyun_queue, aliyun->wiced_aliyun_buffers, aliyun->wiced_xml, aliyun->http_header, option1, option2, option3);
    WICED_VERIFY( send_request( queue_function, request, aliyun->http_client, aliyun->http_header, num_headers, aliyun->wiced_aliyun_buffers ) );
    WICED_VERIFY( reset( aliyun->wiced_aliyun_buffers, aliyun->wiced_xml ) );
    return WICED_SUCCESS;
}

wiced_result_t wiced_xml_start( wiced_xml_t *xml, wiced_aliyun_queue_function_t queue_function )
{
    WICED_VERIFY( wiced_xml_declaration( xml, XML_VERSION_1_0, XML_ENCODING_UTF_8, NULL) );
    switch( queue_function )
    {
    case CREATEQUEUE:
    case SETQUEUEATTRIBUTES:
    case GETQUEUEATTRIBUTES:
    case DELETEQUEUE:
    case LISTQUEUE:
        WICED_VERIFY( wiced_xml_start_attribute( xml, XML_QUEUE, ALIYUN_XMLNS, XML_START_TAG ) );
        break;
    case SENDMESSAGE:
    case RECEIVEMESSAGE:
    case BATCHRECEIVEMESSAGE:
    case DELETEMESSAGE:
    case PEEKMESSAGE:
    case BATCHPEEKMESSAGE:
    case CHANGEMESSAGEVISIBILITY:
        WICED_VERIFY( wiced_xml_start_attribute( xml, XML_MESSAGE, ALIYUN_XMLNS, XML_START_TAG ) );
        break;
    case BATCHDELETEMESSAGE:
        WICED_VERIFY( wiced_xml_start_attribute( xml, XML_RECEIPT_HANDLES, ALIYUN_XMLNS, XML_START_TAG ) );
        break;
    case BATCHSENDMESSAGE:
        WICED_VERIFY( wiced_xml_start_attribute( xml, XML_MESSAGES, ALIYUN_XMLNS, XML_START_TAG ) );
        break;
    }
    return WICED_SUCCESS;
}

wiced_result_t wiced_xml_end( wiced_xml_t *xml, wiced_aliyun_queue_function_t queue_function )
{
    switch( queue_function )
    {
    case CREATEQUEUE:
    case SETQUEUEATTRIBUTES:
    case GETQUEUEATTRIBUTES:
    case DELETEQUEUE:
    case LISTQUEUE:
        WICED_VERIFY( wiced_xml_end_attribute( xml, XML_QUEUE ) );
        break;
    case SENDMESSAGE:
    case RECEIVEMESSAGE:
    case BATCHRECEIVEMESSAGE:
    case DELETEMESSAGE:
    case PEEKMESSAGE:
    case BATCHPEEKMESSAGE:
    case CHANGEMESSAGEVISIBILITY:
        WICED_VERIFY( wiced_xml_end_attribute( xml, XML_MESSAGE ) );
        break;
    case BATCHDELETEMESSAGE:
        WICED_VERIFY( wiced_xml_end_attribute( xml, XML_RECEIPT_HANDLES ) );
        break;
    case BATCHSENDMESSAGE:
        WICED_VERIFY( wiced_xml_end_attribute( xml, XML_MESSAGES ) );
        break;
    }
    return WICED_SUCCESS;
}

wiced_result_t wiced_xml_start_message( wiced_xml_t *xml )
{
    WICED_VERIFY( wiced_xml_start_attribute( xml, XML_MESSAGE, NULL, XML_START_TAG) );
    return WICED_SUCCESS;
}

wiced_result_t wiced_xml_end_message( wiced_xml_t *xml )
{
    WICED_VERIFY( wiced_xml_end_attribute( xml, XML_MESSAGE ) );
    return WICED_SUCCESS;
}

wiced_result_t wiced_xml_add_parameter( wiced_xml_t *xml, char *parameter, char *value )
{
    if( parameter == (char*)XML_MESSAGE_BODY )
    {
        char b64msgbuffer[B64LENGTH(value)];
        wiced_encode_message(value, b64msgbuffer, B64LENGTH(value) );
        WICED_VERIFY( wiced_xml_create_single_element( xml, parameter, b64msgbuffer) );
    }
    else
    {
        WICED_VERIFY( wiced_xml_create_single_element( xml, parameter, value) );
    }

    return WICED_SUCCESS;
}

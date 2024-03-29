
#ifndef CONTROL_UNTRIPLN_CLIENT_HPP
#define CONTROL_UNTRIPLN_CLIENT_HPP

void *setupruntimeresources_wrapper(const std::string & connStr);

void start_wrapper(unsigned int *iter_ptr);

void outputs_wrapper(void *zm, unsigned int *iter_ptr, double *y1_ptr, double *y2_ptr);

void update_wrapper(void *zm, unsigned int *iter_ptr, const double input_data[], unsigned int size );

void terminate_wrapper(void *zm, unsigned int *iter_ptr);

void cleanupruntimeresouces_wrapper(void *zm);

#endif // CONTROL_CLIENT_HPP

#ifndef EXPORT_H_
#define EXPORT_H_

#ifndef UNIX

// #define EXPORT
#ifdef EXPORT
#define BAL_CTRL_API __declspec(dllexport)
#else
#define BAL_CTRL_API __declspec(dllimport)
#endif

#else
#define BAL_CTRL_API
#endif 

#endif
#ifndef LLIST_H_
#define LLIST_H_
#include <iostream>
#include <Node.h>
using namespace std;
namespace CasPlanner
{

class LList
	{
		public :
			Node * 	Start;
		public :
			     	LList();
			     	~LList();
			void 	Add(Node *);
			bool 	Remove(Node *);
			void 	Print();
			void	Free();
			Node *  Find(Node *);
			void   	Next();
			void    Prev();
			Node *  GetHead();
	};

}

#endif /*LLIST_H_*/

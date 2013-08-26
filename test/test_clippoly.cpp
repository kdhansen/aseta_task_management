#include    <cstring>
#include	<iostream>

#include	<clippoly/poly.h>
#include	<clippoly/poly_io.h>
#include	<clippoly/nclip.h>

using namespace ::std;

void
clear(PolyPList &l)
{
	PolyPListIter	i(l);
	while(i())
		delete i.val();
}

int
main(int, char *[])
{
	// Make my own polys
	Poly a(Point(0, 0));
	a.add(Point(1, 0));
	a.add(Point(1, 1));
	a.add(Point(0, 1));

	Poly b(Point(2, 0));
	b.add(Point(3, 0));
	b.add(Point(3, 1));
	b.add(Point(2, 1));

	// Poly	*a = read_poly(cin), *b = read_poly(cin);
	PolyPList	a_min_b, b_min_a, a_and_b;

	// printf("Area a %g b %g\n", a->area(), b->area());

	clip_poly( a, b, a_min_b, b_min_a, a_and_b );

	if(a_and_b.empty())
		cout << "No overlap" << endl;

	// cout << "a_min_b:\n" << a_min_b;
	// cout << "b_min_a:\n" << b_min_a;
	// cout << "a_and_b:\n" << a_and_b;

	// delete	a;
	// delete	b;

	clear(a_min_b);
	clear(b_min_a);
	clear(a_and_b);

	return 0;
}
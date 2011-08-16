/*
 *    Copyright (c) 1996 Paul Mackerras <paulus@cs.anu.edu.au>
 *      Changes to accomodate Power Macintoshes.
 *    Cort Dougan <cort@cs.nmt.edu>
 *      Rewrites.
 *    Grant Erickson <grant@lcse.umn.edu>
 *      General rework and split from mm/init.c.
 *
 *    Module name: mem_pieces.c
 *
 *    Description:
 *      Routines and data structures for manipulating and representing
 *      phyiscal memory extents (i.e. address/length pairs).
 *
 */

#include <appleboot-mem_pieces.h>

extern struct mem_pieces phys_mem;

/*
 * Scan a region for a piece of a given size with the required alignment.
 */
unsigned int
mem_pieces_find(unsigned int size, unsigned int align)
{
	int i;
	unsigned int a, e;
	struct mem_pieces *mp = &phys_mem;

	for (i = 0; i < mp->n_regions; ++i) {
		a = mp->regions[i].address;
		e = a + mp->regions[i].size;
		a = (a + align - 1) & -align;
		if (a + size <= e) {
			mem_pieces_remove(mp, a, size, 1);
			return a;
		}
	}
	
	return 0xffffffff;
}

/*
 * Remove some memory from an array of pieces
 */
void
mem_pieces_remove(struct mem_pieces *mp, unsigned int start, unsigned int size,
		  int must_exist)
{
	int i, j;
	unsigned int end, rs, re;
	struct reg_property *rp;

	end = start + size;
	for (i = 0, rp = mp->regions; i < mp->n_regions; ++i, ++rp) {
		if (end > rp->address && start < rp->address + rp->size)
			break;
	}
	if (i >= mp->n_regions)
		return;
	for (; i < mp->n_regions && end > rp->address; ++i, ++rp) {
		rs = rp->address;
		re = rs + rp->size;
		if (start > rs) {
			rp->size = start - rs;
			if (end < re) {
				/* need to split this entry */
				for (j = mp->n_regions; j > i + 1; --j) {
					mp->regions[j].address
						= mp->regions[j-1].address;
					mp->regions[j].size
						= mp->regions[j-1].size;
				}
				++mp->n_regions;
				rp[1].address = end;
				rp[1].size = re - end;
			}
		} else {
			if (end < re) {
				rp->address = end;
				rp->size = re - end;
			} else {
				/* need to delete this entry */
				for (j = i; j < mp->n_regions - 1; ++j)
					mp->regions[j] = mp->regions[j+1];
				--mp->n_regions;
				--i;
				--rp;
			}
		}
	}
}

/*
 * Add some memory to an array of pieces
 */
void
mem_pieces_append(struct mem_pieces *mp, unsigned int start, unsigned int size)
{
	struct reg_property *rp;

	if (mp->n_regions >= MEM_PIECES_MAX)
		return;
	rp = &mp->regions[mp->n_regions++];
	rp->address = start;
	rp->size = size;
}

void
mem_pieces_sort(struct mem_pieces *mp)
{
	unsigned long a, s;
	int i, j;

	for (i = 1; i < mp->n_regions; ++i) {
		a = mp->regions[i].address;
		s = mp->regions[i].size;
		for (j = i - 1; j >= 0; --j) {
			if (a >= mp->regions[j].address)
				break;
			mp->regions[j+1] = mp->regions[j];
		}
		mp->regions[j+1].address = a;
		mp->regions[j+1].size = s;
	}
}

void
mem_pieces_coalesce(struct mem_pieces *mp)
{
	unsigned long a, s, ns;
	int i, j, d;

	d = 0;
	for (i = 0; i < mp->n_regions; i = j) {
		a = mp->regions[i].address;
		s = mp->regions[i].size;
		for (j = i + 1; j < mp->n_regions
			     && mp->regions[j].address - a <= s; ++j) {
			ns = mp->regions[j].address + mp->regions[j].size - a;
			if (ns > s)
				s = ns;
		}
		mp->regions[d].address = a;
		mp->regions[d].size = s;
		++d;
	}
	mp->n_regions = d;
}

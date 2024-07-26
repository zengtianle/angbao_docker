/*-
 * Copyright (c) 2008 Joerg Sonnenberger
 * Copyright (c) 2009-2012 Michihiro NAKAJIMA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "archive_platform.h"
__FBSDID("$FreeBSD: head/lib/libarchive/archive_write_set_format_mtree.c 201171 2009-12-29 06:39:07Z kientzle $");

#ifdef HAVE_SYS_TYPES_H
#include <sys/types.h>
#endif
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "archive.h"
#include "archive_digest_private.h"
#include "archive_entry.h"
#include "archive_entry_private.h"
#include "archive_private.h"
#include "archive_rb.h"
#include "archive_string.h"
#include "archive_write_private.h"

#define INDENTNAMELEN	15
#define MAXLINELEN	80
#define SET_KEYS	\
	(F_FLAGS | F_GID | F_GNAME | F_MODE | F_TYPE | F_UID | F_UNAME)

struct attr_counter {
	struct attr_counter *prev;
	struct attr_counter *next;
	struct mtree_entry *m_entry;
	int count;
};

struct att_counter_set {
	struct attr_counter *uid_list;
	struct attr_counter *gid_list;
	struct attr_counter *mode_list;
	struct attr_counter *flags_list;
};

struct mtree_chain {
	struct mtree_entry *first;
	struct mtree_entry **last;
};

/*
 * The Data only for a directory file.
 */
struct dir_info {
	struct archive_rb_tree rbtree;
	struct mtree_chain children;
	struct mtree_entry *chnext;
	int virtual;
};

/*
 * The Data only for a regular file.
 */
struct reg_info {
	int compute_sum;
	uint32_t crc;
	struct ae_digest digest;
};

struct mtree_entry {
	struct archive_rb_node rbnode;
	struct mtree_entry *next;
	struct mtree_entry *parent;
	struct dir_info *dir_info;
	struct reg_info *reg_info;

	struct archive_string parentdir;
	struct archive_string basename;
	struct archive_string pathname;
	struct archive_string symlink;
	struct archive_string uname;
	struct archive_string gname;
	struct archive_string fflags_text;
	unsigned int nlink;
	mode_t filetype;
	mode_t mode;
	int64_t size;
	int64_t uid;
	int64_t gid;
	time_t mtime;
	long mtime_nsec;
	unsigned long fflags_set;
	unsigned long fflags_clear;
	dev_t rdevmajor;
	dev_t rdevminor;
	dev_t devmajor;
	dev_t devminor;
	int64_t ino;
};

struct mtree_writer {
	struct mtree_entry *mtree_entry;
	struct mtree_entry *root;
	struct mtree_entry *cur_dirent;
	struct archive_string cur_dirstr;
	struct mtree_chain file_list;

	struct archive_string ebuf;
	struct archive_string buf;
	int first;
	uint64_t entry_bytes_remaining;

	/*
	 * Set global value.
	 */
	struct {
		int		processing;
		mode_t		type;
		int		keys;
		int64_t		uid;
		int64_t		gid;
		mode_t		mode;
		unsigned long	fflags_set;
		unsigned long	fflags_clear;
	} set;
	struct att_counter_set	acs;
	int classic;
	int depth;

	/* check sum */
	int compute_sum;
	uint32_t crc;
	uint64_t crc_len;
#ifdef ARCHIVE_HAS_MD5
	archive_md5_ctx md5ctx;
#endif
#ifdef ARCHIVE_HAS_RMD160
	archive_rmd160_ctx rmd160ctx;
#endif
#ifdef ARCHIVE_HAS_SHA1
	archive_sha1_ctx sha1ctx;
#endif
#ifdef ARCHIVE_HAS_SHA256
	archive_sha256_ctx sha256ctx;
#endif
#ifdef ARCHIVE_HAS_SHA384
	archive_sha384_ctx sha384ctx;
#endif
#ifdef ARCHIVE_HAS_SHA512
	archive_sha512_ctx sha512ctx;
#endif
	/* Keyword options */
	int keys;
#define	F_CKSUM		0x00000001		/* checksum */
#define	F_DEV		0x00000002		/* device type */
#define	F_DONE		0x00000004		/* directory done */
#define	F_FLAGS		0x00000008		/* file flags */
#define	F_GID		0x00000010		/* gid */
#define	F_GNAME		0x00000020		/* group name */
#define	F_IGN		0x00000040		/* ignore */
#define	F_MAGIC		0x00000080		/* name has magic chars */
#define	F_MD5		0x00000100		/* MD5 digest */
#define	F_MODE		0x00000200		/* mode */
#define	F_NLINK		0x00000400		/* number of links */
#define	F_NOCHANGE 	0x00000800		/* If owner/mode "wrong", do
						 * not change */
#define	F_OPT		0x00001000		/* existence optional */
#define	F_RMD160 	0x00002000		/* RIPEMD160 digest */
#define	F_SHA1		0x00004000		/* SHA-1 digest */
#define	F_SIZE		0x00008000		/* size */
#define	F_SLINK		0x00010000		/* symbolic link */
#define	F_TAGS		0x00020000		/* tags */
#define	F_TIME		0x00040000		/* modification time */
#define	F_TYPE		0x00080000		/* file type */
#define	F_UID		0x00100000		/* uid */
#define	F_UNAME		0x00200000		/* user name */
#define	F_VISIT		0x00400000		/* file visited */
#define	F_SHA256	0x00800000		/* SHA-256 digest */
#define	F_SHA384	0x01000000		/* SHA-384 digest */
#define	F_SHA512	0x02000000		/* SHA-512 digest */
#define	F_INO		0x04000000		/* inode number */
#define	F_RESDEV	0x08000000		/* device ID on which the
						 * entry resides */

	/* Options */
	int dironly;		/* If it is set, ignore all files except
				 * directory files, like mtree(8) -d option. */
	int indent;		/* If it is set, indent output data. */
	int output_global_set;	/* If it is set, use /set keyword to set
				 * global values. When generating mtree
				 * classic format, it is set by default. */
};

#define DEFAULT_KEYS	(F_DEV | F_FLAGS | F_GID | F_GNAME | F_SLINK | F_MODE\
			 | F_NLINK | F_SIZE | F_TIME | F_TYPE | F_UID\
			 | F_UNAME)
#define attr_counter_set_reset	attr_counter_set_free

static void attr_counter_free(struct attr_counter **);
static int attr_counter_inc(struct attr_counter **, struct attr_counter *,
	struct attr_counter *, struct mtree_entry *);
static struct attr_counter * attr_counter_new(struct mtree_entry *,
	struct attr_counter *);
static int attr_counter_set_collect(struct mtree_writer *,
	struct mtree_entry *);
static void attr_counter_set_free(struct mtree_writer *);
static int get_global_set_keys(struct mtree_writer *, struct mtree_entry *);
static int mtree_entry_add_child_tail(struct mtree_entry *,
	struct mtree_entry *);
static int mtree_entry_create_virtual_dir(struct archive_write *, const char *,
	struct mtree_entry **);
static int mtree_entry_cmp_node(const struct archive_rb_node *,
	const struct archive_rb_node *);
static int mtree_entry_cmp_key(const struct archive_rb_node *, const void *);
static int mtree_entry_exchange_same_entry(struct archive_write *,
    struct mtree_entry *, struct mtree_entry *);
static void mtree_entry_free(struct mtree_entry *);
static int mtree_entry_new(struct archive_write *, struct archive_entry *,
	struct mtree_entry **);
static void mtree_entry_register_free(struct mtree_writer *);
static void mtree_entry_register_init(struct mtree_writer *);
static int mtree_entry_setup_filenames(struct archive_write *,
	struct mtree_entry *, struct archive_entry *);
static int mtree_entry_tree_add(struct archive_write *, struct mtree_entry **);
static void sum_init(struct mtree_writer *);
static void sum_update(struct mtree_writer *, const void *, size_t);
static void sum_final(struct mtree_writer *, struct reg_info *);
static void sum_write(struct archive_string *, struct reg_info *);
static int write_mtree_entry(struct archive_write *, struct mtree_entry *);
static int write_dot_dot_entry(struct archive_write *, struct mtree_entry *);

#define	COMPUTE_CRC(var, ch)	(var) = (var) << 8 ^ crctab[(var) >> 24 ^ (ch)]
static const uint32_t crctab[] = {
	0x0,
	0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
	0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6,
	0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
	0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac,
	0x5bd4b01b, 0x569796c2, 0x52568b75, 0x6a1936c8, 0x6ed82b7f,
	0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a,
	0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
	0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58,
	0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033,
	0xa4ad16ea, 0xa06c0b5d, 0xd4326d90, 0xd0f37027, 0xddb056fe,
	0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
	0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4,
	0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
	0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5,
	0x2ac12072, 0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16,
	0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca, 0x7897ab07,
	0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c,
	0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1,
	0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
	0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b,
	0xbb60adfc, 0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698,
	0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d,
	0x94ea7b2a, 0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e,
	0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2, 0xc6bcf05f,
	0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
	0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80,
	0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
	0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a,
	0x58c1663d, 0x558240e4, 0x51435d53, 0x251d3b9e, 0x21dc2629,
	0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c,
	0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
	0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e,
	0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65,
	0xeba91bbc, 0xef68060b, 0xd727bbb6, 0xd3e6a601, 0xdea580d8,
	0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
	0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2,
	0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
	0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74,
	0x857130c3, 0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640,
	0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c, 0x7b827d21,
	0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a,
	0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e, 0x18197087,
	0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
	0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d,
	0x2056cd3a, 0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce,
	0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb,
	0xdbee767c, 0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18,
	0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4, 0x89b8fd09,
	0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
	0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf,
	0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};

static const unsigned char safe_char[256] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 00 - 0F */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 10 - 1F */
	/* !"$%&'()*+,-./  EXCLUSION:0x20( ) 0x23(#) */
	0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, /* 20 - 2F */
	/* 0123456789:;<>?  EXCLUSION:0x3d(=) */
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, /* 30 - 3F */
	/* @ABCDEFGHIJKLMNO */
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, /* 40 - 4F */
	/* PQRSTUVWXYZ[]^_ EXCLUSION:0x5c(\)  */
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, /* 50 - 5F */
	/* `abcdefghijklmno */
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, /* 60 - 6F */
	/* pqrstuvwxyz{|}~ */
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, /* 70 - 7F */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 80 - 8F */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* 90 - 9F */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* A0 - AF */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* B0 - BF */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* C0 - CF */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* D0 - DF */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* E0 - EF */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, /* F0 - FF */
};

static void
mtree_quote(struct archive_string *s, const char *str)
{
	const char *start;
	char buf[4];
	unsigned char c;

	for (start = str; *str != '\0'; ++str) {
		if (safe_char[*(const unsigned char *)str])
			continue;
		if (start != str)
			archive_strncat(s, start, str - start);
		c = (unsigned char)*str;
		buf[0] = '\\';
		buf[1] = (c / 64) + '0';
		buf[2] = (c / 8 % 8) + '0';
		buf[3] = (c % 8) + '0';
		archive_strncat(s, buf, 4);
		start = str + 1;
	}

	if (start != str)
		archive_strncat(s, start, str - start);
}

/*
 * Indent a line as the mtree utility does so it is readable for people.
 */
static void
mtree_indent(struct mtree_writer *mtree)
{
	int i, fn, nd, pd;
	const char *r, *s, *x;

	if (mtree->classic) {
		if (mtree->indent) {
			nd = 0;
			pd = mtree->depth * 4;
		} else {
			nd = mtree->depth?4:0;
			pd = 0;
		}
	} else
		nd = pd = 0;
	fn = 1;
	s = r = mtree->ebuf.s;
	x = NULL;
	while (*r == ' ')
		r++;
	while ((r = strchr(r, ' ')) != NULL) {
		if (fn) {
			fn = 0;
			for (i = 0; i < nd + pd; i++)
				archive_strappend_char(&mtree->buf, ' ');
			archive_strncat(&mtree->buf, s, r - s);
			if (nd + (r -s) > INDENTNAMELEN) {
				archive_strncat(&mtree->buf, " \\\n", 3);
				for (i = 0; i < (INDENTNAMELEN + 1 + pd); i++)
					archive_strappend_char(&mtree->buf, ' ');
			} else {
				for (i = (int)(r -s + nd);
				    i < (INDENTNAMELEN + 1); i++)
					archive_strappend_char(&mtree->buf, ' ');
			}
			s = ++r;
			x = NULL;
			continue;
		}
		if (pd + (r - s) <= MAXLINELEN - 3 - INDENTNAMELEN)
			x = r++;
		else {
			if (x == NULL)
				x = r;
			archive_strncat(&mtree->buf, s, x - s);
			archive_strncat(&mtree->buf, " \\\n", 3);
			for (i = 0; i < (INDENTNAMELEN + 1 + pd); i++)
				archive_strappend_char(&mtree->buf, ' ');
			s = r = ++x;
			x = NULL;
		}
	}
	if (fn) {
		for (i = 0; i < nd + pd; i++)
			archive_strappend_char(&mtree->buf, ' ');
		archive_strcat(&mtree->buf, s);
		s += strlen(s);
	}
	if (x != NULL && pd + strlen(s) > MAXLINELEN - 3 - INDENTNAMELEN) {
		/* Last keyword is longer. */
		archive_strncat(&mtree->buf, s, x - s);
		archive_strncat(&mtree->buf, " \\\n", 3);
		for (i = 0; i < (INDENTNAMELEN + 1 + pd); i++)
			archive_strappend_char(&mtree->buf, ' ');
		s = ++x;
	}
	archive_strcat(&mtree->buf, s);
	archive_string_empty(&mtree->ebuf);
}

/*
 * Write /set keyword.
 * Set the most used value of uid, gid, mode and fflags, which are
 * collected by the attr_counter_set_collect() function.
 */
static void
write_global(struct mtree_writer *mtree)
{
	struct archive_string setstr;
	struct archive_string unsetstr;
	struct att_counter_set *acs;
	int keys, oldkeys, effkeys;

	archive_string_init(&setstr);
	archive_string_init(&unsetstr);
	keys = mtree->keys & SET_KEYS;
	oldkeys = mtree->set.keys;
	effkeys = keys;
	acs = &mtree->acs;
	if (mtree->set.processing) {
		/*
		 * Check if the global data needs updating.
		 */
		effkeys &= ~F_TYPE;
		if (acs->uid_list == NULL)
			effkeys &= ~(F_UNAME | F_UID);
		else if (oldkeys & (F_UNAME | F_UID)) {
			if (acs->uid_list->count < 2 ||
			    mtree->set.uid == acs->uid_list->m_entry->uid)
				effkeys &= ~(F_UNAME | F_UID);
		}
		if (acs->gid_list == NULL)
			effkeys &= ~(F_GNAME | F_GID);
		else if (oldkeys & (F_GNAME | F_GID)) {
			if (acs->gid_list->count < 2 ||
			    mtree->set.gid == acs->gid_list->m_entry->gid)
				effkeys &= ~(F_GNAME | F_GID);
		}
		if (acs->mode_list == NULL)
			effkeys &= ~F_MODE;
		else if (oldkeys & F_MODE) {
			if (acs->mode_list->count < 2 ||
			    mtree->set.mode == acs->mode_list->m_entry->mode)
				effkeys &= ~F_MODE;
		}
		if (acs->flags_list == NULL)
			effkeys &= ~F_FLAGS;
		else if ((oldkeys & F_FLAGS) != 0) {
			if (acs->flags_list->count < 2 ||
			    (acs->flags_list->m_entry->fflags_set ==
				mtree->set.fflags_set &&
			     acs->flags_list->m_entry->fflags_clear ==
				mtree->set.fflags_clear))
				effkeys &= ~F_FLAGS;
		}
	} else {
		if (acs->uid_list == NULL)
			keys &= ~(F_UNAME | F_UID);
		if (acs->gid_list == NULL)
			keys &= ~(F_GNAME | F_GID);
		if (acs->mode_list == NULL)
			keys &= ~F_MODE;
		if (acs->flags_list == NULL)
			keys &= ~F_FLAGS;
	}
	if ((keys & effkeys & F_TYPE) != 0) {
		if (mtree->dironly) {
			archive_strcat(&setstr, " type=dir");
			mtree->set.type = AE_IFDIR;
		} else {
			archive_strcat(&setstr, " type=file");
			mtree->set.type = AE_IFREG;
		}
	}
	if ((keys & effkeys & F_UNAME) != 0) {
		if (archive_strlen(&(acs->uid_list->m_entry->uname)) > 0) {
			archive_strcat(&setstr, " uname=");
			mtree_quote(&setstr, acs->uid_list->m_entry->uname.s);
		} else {
			keys &= ~F_UNAME;
			if ((oldkeys & F_UNAME) != 0)
				archive_strcat(&unsetstr, " uname");
		}
	}
	if ((keys & effkeys & F_UID) != 0) {
		mtree->set.uid = acs->uid_list->m_entry->uid;
		archive_string_sprintf(&setstr, " uid=%jd",
		    (intmax_t)mtree->set.uid);
	}
	if ((keys & effkeys & F_GNAME) != 0) {
		if (archive_strlen(&(acs->gid_list->m_entry->gname)) > 0) {
			archive_strcat(&setstr, " gname=");
			mtree_quote(&setstr, acs->gid_list->m_entry->gname.s);
		} else {
			keys &= ~F_GNAME;
			if ((oldkeys & F_GNAME) != 0)
				archive_strcat(&unsetstr, " gname");
		}
	}
	if ((keys & effkeys & F_GID) != 0) {
		mtree->set.gid = acs->gid_list->m_entry->gid;
		archive_string_sprintf(&setstr, " gid=%jd",
		    (intmax_t)mtree->set.gid);
	}
	if ((keys & effkeys & F_MODE) != 0) {
		mtree->set.mode = acs->mode_list->m_entry->mode;
		archive_string_sprintf(&setstr, " mode=%o",
		    (unsigned int)mtree->set.mode);
	}
	if ((keys & effkeys & F_FLAGS) != 0) {
		if (archive_strlen(
		    &(acs->flags_list->m_entry->fflags_text)) > 0) {
			archive_strcat(&setstr, " flags=");
			mtree_quote(&setstr,
			    acs->flags_list->m_entry->fflags_text.s);
			mtree->set.fflags_set =
			    acs->flags_list->m_entry->fflags_set;
			mtree->set.fflags_clear =
			    acs->flags_list->m_entry->fflags_clear;
		} else {
			keys &= ~F_FLAGS;
			if ((oldkeys & F_FLAGS) != 0)
				archive_strcat(&unsetstr, " flags");
		}
	}
	if (unsetstr.length > 0)
		archive_string_sprintf(&mtree->buf, "/unset%s\n", unsetstr.s);
	archive_string_free(&unsetstr);
	if (setstr.length > 0)
		archive_string_sprintf(&mtree->buf, "/set%s\n", setstr.s);
	archive_string_free(&setstr);
	mtree->set.keys = keys;
	mtree->set.processing = 1;
}

static struct attr_counter *
attr_counter_new(struct mtree_entry *me, struct attr_counter *prev)
{
	struct attr_counter *ac;

	ac = malloc(sizeof(*ac));
	if (ac != NULL) {
		ac->prev = prev;
		ac->next = NULL;
		ac->count = 1;
		ac->m_entry = me;
	}
	return (ac);
}

static void
attr_counter_free(struct attr_counter **top)
{
	struct attr_counter *ac, *tac;

	if (*top == NULL)
		return;
	ac = *top;
        while (ac != NULL) {
		tac = ac->next;
		free(ac);
		ac = tac;
	}
	*top = NULL;
}

static int
attr_counter_inc(struct attr_counter **top, struct attr_counter *ac,
    struct attr_counter *last, struct mtree_entry *me)
{
	struct attr_counter *pac;

	if (ac != NULL) {
		ac->count++;
		if (*top == ac || ac->prev->count >= ac->count)
			return (0);
		for (pac = ac->prev; pac; pac = pac->prev) {
			if (pac->count >= ac->count)
				break;
		}
		ac->prev->next = ac->next;
		if (ac->next != NULL)
			ac->next->prev = ac->prev;
		if (pac != NULL) {
			ac->prev = pac;
			ac->next = pac->next;
			pac->next = ac;
			if (ac->next != NULL)
				ac->next->prev = ac;
		} else {
			ac->prev = NULL;
			ac->next = *top;
			*top = ac;
			ac->next->prev = ac;
		}
	} else if (last != NULL) {
		ac = attr_counter_new(me, last);
		if (ac == NULL)
			return (-1);
		last->next = ac;
	}
	return (0);
}

/*
 * Tabulate uid, gid, mode and fflags of a entry in order to be used for /set.
 */
static int
attr_counter_set_collect(struct mtree_writer *mtree, struct mtree_entry *me)
{
	struct attr_counter *ac, *last;
	struct att_counter_set *acs = &mtree->acs;
	int keys = mtree->keys;

	if (keys & (F_UNAME | F_UID)) {
		if (acs->uid_list == NULL) {
			acs->uid_list = attr_counter_new(me, NULL);
			if (acs->uid_list == NULL)
				return (-1);
		} else {
			last = NULL;
			for (ac = acs->uid_list; ac; ac = ac->next) {
				if (ac->m_entry->uid == me->uid)
					break;
				last = ac;
			}
			if (attr_counter_inc(&acs->uid_list, ac, last, me) < 0)
				return (-1);
		}
	}
	if (keys & (F_GNAME | F_GID)) {
		if (acs->gid_list == NULL) {
			acs->gid_list = attr_counter_new(me, NULL);
			if (acs->gid_list == NULL)
				return (-1);
		} else {
			last = NULL;
			for (ac = acs->gid_list; ac; ac = ac->next) {
				if (ac->m_entry->gid == me->gid)
					break;
				last = ac;
			}
			if (attr_counter_inc(&acs->gid_list, ac, last, me) < 0)
				return (-1);
		}
	}
	if (keys & F_MODE) {
		if (acs->mode_list == NULL) {
			acs->mode_list = attr_counter_new(me, NULL);
			if (acs->mode_list == NULL)
				return (-1);
		} else {
			last = NULL;
			for (ac = acs->mode_list; ac; ac = ac->next) {
				if (ac->m_entry->mode == me->mode)
					break;
				last = ac;
			}
			if (attr_counter_inc(&acs->mode_list, ac, last, me) < 0)
				return (-1);
		}
	}
	if (keys & F_FLAGS) {
		if (acs->flags_list == NULL) {
			acs->flags_list = attr_counter_new(me, NULL);
			if (acs->flags_list == NULL)
				return (-1);
		} else {
			last = NULL;
			for (ac = acs->flags_list; ac; ac = ac->next) {
				if (ac->m_entry->fflags_set == me->fflags_set &&
				    ac->m_entry->fflags_clear ==
							me->fflags_clear)
					break;
				last = ac;
			}
			if (attr_counter_inc(&acs->flags_list, ac, last, me) < 0)
				return (-1);
		}
	}

	return (0);
}

static void
attr_counter_set_free(struct mtree_writer *mtree)
{
	struct att_counter_set *acs = &mtree->acs;

	attr_counter_free(&acs->uid_list);
	attr_counter_free(&acs->gid_list);
	attr_counter_free(&acs->mode_list);
	attr_counter_free(&acs->flags_list);
}

static int
get_global_set_keys(struct mtree_writer *mtree, struct mtree_entry *me)
{
	int keys;

	keys = mtree->keys;

	/*
	 * If a keyword has been set by /set, we do not need to
	 * output it.
	 */
	if (mtree->set.keys == 0)
		return (keys);/* /set is not used. */

	if ((mtree->set.keys & (F_GNAME | F_GID)) != 0 &&
	     mtree->set.gid == me->gid)
		keys &= ~(F_GNAME | F_GID);
	if ((mtree->set.keys & (F_UNAME | F_UID)) != 0 &&
	     mtree->set.uid == me->uid)
		keys &= ~(F_UNAME | F_UID);
	if (mtree->set.keys & F_FLAGS) {
		if (mtree->set.fflags_set == me->fflags_set &&
		    mtree->set.fflags_clear == me->fflags_clear)
			keys &= ~F_FLAGS;
	}
	if ((mtree->set.keys & F_MODE) != 0 && mtree->set.mode == me->mode)
		keys &= ~F_MODE;

	switch (me->filetype) {
	case AE_IFLNK: case AE_IFSOCK: case AE_IFCHR:
	case AE_IFBLK: case AE_IFIFO:
		break;
	case AE_IFDIR:
		if ((mtree->set.keys & F_TYPE) != 0 &&
		    mtree->set.type == AE_IFDIR)
			keys &= ~F_TYPE;
		break;
	case AE_IFREG:
	default:	/* Handle unknown file types as regular files. */
		if ((mtree->set.keys & F_TYPE) != 0 &&
		    mtree->set.type == AE_IFREG)
			keys &= ~F_TYPE;
		break;
	}

	return (keys);
}

static int
mtree_entry_new(struct archive_write *a, struct archive_entry *entry,
    struct mtree_entry **m_entry)
{
	struct mtree_entry *me;
	const char *s;
	int r;
	static const struct archive_rb_tree_ops rb_ops = {
		mtree_entry_cmp_node, mtree_entry_cmp_key
	};

	me = calloc(1, sizeof(*me));
	if (me == NULL) {
		archive_set_error(&a->archive, ENOMEM,
		    "Can't allocate memory for a mtree entry");
		*m_entry = NULL;
		return (ARCHIVE_FATAL);
	}

	r = mtree_entry_setup_filenames(a, me, entry);
	if (r < ARCHIVE_WARN) {
		mtree_entry_free(me);
		*m_entry = NULL;
		return (r);
	}

	if ((s = archive_entry_symlink(entry)) != NULL)
		archive_strcpy(&me->symlink, s);
	me->nlink = archive_entry_nlink(entry);
	me->filetype = archive_entry_filetype(entry);
	me->mode = archive_entry_mode(entry) & 07777;
	me->uid = archive_entry_uid(entry);
	me->gid = archive_entry_gid(entry);
	if ((s = archive_entry_uname(entry)) != NULL)
		archive_strcpy(&me->uname, s);
	if ((s = archive_entry_gname(entry)) != NULL)
		archive_strcpy(&me->gname, s);
	if ((s = archive_entry_fflags_text(entry)) != NULL)
		archive_strcpy(&me->fflags_text, s);
	archive_entry_fflags(entry, &me->fflags_set, &me->fflags_clear);
	me->mtime = archive_entry_mtime(entry);
	me->mtime_nsec = archive_entry_mtime_nsec(entry);
	me->rdevmajor = archive_entry_rdevmajor(entry);
	me->rdevminor = archive_entry_rdevminor(entry);
	me->devmajor = archive_entry_devmajor(entry);
	me->devminor = archive_entry_devminor(entry);
	me->ino = archive_entry_ino(entry);
	me->size = archive_entry_size(entry);
	if (me->filetype == AE_IFDIR) {
		me->dir_info = calloc(1, sizeof(*me->dir_info));
		if (me->dir_info == NULL) {
			mtree_entry_free(me);
			archive_set_error(&a->archive, ENOMEM,
			    "Can't allocate memory for a mtree entry");
			*m_entry = NULL;
			return (ARCHIVE_FATAL);
		}
		__archive_rb_tree_init(&me->dir_info->rbtree, &rb_ops);
		me->dir_info->children.first = NULL;
		me->dir_info->children.last = &(me->dir_info->children.first);
		me->dir_info->chnext = NULL;
	} else if (me->filetype == AE_IFREG) {
		me->reg_info = calloc(1, sizeof(*me->reg_info));
		if (me->reg_info == NULL) {
			mtree_entry_free(me);
			archive_set_error(&a->archive, ENOMEM,
			    "Can't allocate memory for a mtree entry");
			*m_entry = NULL;
			return (ARCHIVE_FATAL);
		}
		me->reg_info->compute_sum = 0;
	}

	*m_entry = me;
	return (ARCHIVE_OK);
}

static void
mtree_entry_free(struct mtree_entry *me)
{
	archive_string_free(&me->parentdir);
	archive_string_free(&me->basename);
	archive_string_free(&me->pathname);
	archive_string_free(&me->symlink);
	archive_string_free(&me->uname);
	archive_string_free(&me->gname);
	archive_string_free(&me->fflags_text);
	free(me->dir_info);
	free(me->reg_info);
	free(me);
}

static int
archive_write_mtree_header(struct archive_write *a,
    struct archive_entry *entry)
{
	struct mtree_writer *mtree= a->format_data;
	struct mtree_entry *mtree_entry;
	int r, r2;

	if (mtree->first) {
		mtree->first = 0;
		archive_strcat(&mtree->buf, "#mtree\n");
		if ((mtree->keys & SET_KEYS) == 0)
			mtree->output_global_set = 0;/* Disabled. */
	}

	mtree->entry_bytes_remaining = archive_entry_size(entry);

	/* While directory only mode, we do not handle non directory files. */
	if (mtree->dironly && archive_entry_filetype(entry) != AE_IFDIR)
		return (ARCHIVE_OK);

	r2 = mtree_entry_new(a, entry, &mtree_entry);
	if (r2 < ARCHIVE_WARN)
		return (r2);
	r = mtree_entry_tree_add(a, &mtree_entry);
	if (r < ARCHIVE_WARN) {
		mtree_entry_free(mtree_entry);
		return (r);
	}
	mtree->mtree_entry = mtree_entry;

	/* If the current file is a regular file, we have to
	 * compute the sum of its content.
	 * Initialize a bunch of checksum context. */
	if (mtree_entry->reg_info)
		sum_init(mtree);

	return (r2);
}

static int
write_mtree_entry(struct archive_write *a, struct mtree_entry *me)
{
	struct mtree_writer *mtree = a->format_data;
	struct archive_string *str;
	int keys, ret;

	if (me->dir_info) {
		if (mtree->classic) {
			/*
			 * Output a comment line to describe the full
			 * pathname of the entry as mtree utility does
			 * while generating classic format.
			 */
			if (!mtree->dironly)
				archive_strappend_char(&mtree->buf, '\n');
			if (me->parentdir.s)
				archive_string_sprintf(&mtree->buf,
				    "# %s/%s\n",
				    me->parentdir.s, me->basename.s);
			else
				archive_string_sprintf(&mtree->buf,
				    "# %s\n",
				    me->basename.s);
		}
		if (mtree->output_global_set)
			write_global(mtree);
	}
	archive_string_empty(&mtree->ebuf);
	str = (mtree->indent || mtree->classic)? &mtree->ebuf : &mtree->buf;

	if (!mtree->classic && me->parentdir.s) {
		/*
		 * If generating format is not classic one(v1), output
		 * a full pathname.
		 */
		mtree_quote(str, me->parentdir.s);
		archive_strappend_char(str, '/');
	}
	mtree_quote(str, me->basename.s);

	keys = get_global_set_keys(mtree, me);
	if ((keys & F_NLINK) != 0 &&
	    me->nlink != 1 && me->filetype != AE_IFDIR)
		archive_string_sprintf(str, " nlink=%u", me->nlink);

	if ((keys & F_GNAME) != 0 && archive_strlen(&me->gname) > 0) {
		archive_strcat(str, " gname=");
		mtree_quote(str, me->gname.s);
	}
	if ((keys & F_UNAME) != 0 && archive_strlen(&me->uname) > 0) {
		archive_strcat(str, " uname=");
		mtree_quote(str, me->uname.s);
	}
	if ((keys & F_FLAGS) != 0) {
		if (archive_strlen(&me->fflags_text) > 0) {
			archive_strcat(str, " flags=");
			mtree_quote(str, me->fflags_text.s);
		} else if (mtree->set.processing &&
		    (mtree->set.keys & F_FLAGS) != 0)
			/* Overwrite the global parameter. */
			archive_strcat(str, " flags=none");
	}
	if ((keys & F_TIME) != 0)
		archive_string_sprintf(str, " time=%jd.%jd",
		    (intmax_t)me->mtime, (intmax_t)me->mtime_nsec);
	if ((keys & F_MODE) != 0)
		archive_string_sprintf(str, " mode=%o", (unsigned int)me->mode);
	if ((keys & F_GID) != 0)
		archive_string_sprintf(str, " gid=%jd", (intmax_t)me->gid);
	if ((keys & F_UID) != 0)
		archive_string_sprintf(str, " uid=%jd", (intmax_t)me->uid);

	if ((keys & F_INO) != 0)
		archive_string_sprintf(str, " inode=%jd", (intmax_t)me->ino);
	if ((keys & F_RESDEV) != 0) {
		archive_string_sprintf(str,
		    " resdevice=native,%ju,%ju",
		    (uintmax_t)me->devmajor,
		    (uintmax_t)me->devminor);
	}

	switch (me->filetype) {
	case AE_IFLNK:
		if ((keys & F_TYPE) != 0)
			archive_strcat(str, " type=link");
		if ((keys & F_SLINK) != 0) {
			archive_strcat(str, " link=");
			mtree_quote(str, me->symlink.s);
		}
		break;
	case AE_IFSOCK:
		if ((keys & F_TYPE) != 0)
			archive_strcat(str, " type=socket");
		break;
	case AE_IFCHR:
		if ((keys & F_TYPE) != 0)
			archive_strcat(str, " type=char");
		if ((keys & F_DEV) != 0) {
			archive_string_sprintf(str,
			    " device=native,%ju,%ju",
			    (uintmax_t)me->rdevmajor,
			    (uintmax_t)me->rdevminor);
		}
		break;
	case AE_IFBLK:
		if ((keys & F_TYPE) != 0)
			archive_strcat(str, " type=block");
		if ((keys & F_DEV) != 0) {
			archive_string_sprintf(str,
			    " device=native,%ju,%ju",
			    (uintmax_t)me->rdevmajor,
			    (uintmax_t)me->rdevminor);
		}
		break;
	case AE_IFDIR:
		if ((keys & F_TYPE) != 0)
			archive_strcat(str, " type=dir");
		break;
	case AE_IFIFO:
		if ((keys & F_TYPE) != 0)
			archive_strcat(str, " type=fifo");
		break;
	case AE_IFREG:
	default:	/* Handle unknown file types as regular files. */
		if ((keys & F_TYPE) != 0)
			archive_strcat(str, " type=file");
		if ((keys & F_SIZE) != 0)
			archive_string_sprintf(str, " size=%jd",
			    (intmax_t)me->size);
		break;
	}

	/* Write a bunch of sum. */
	if (me->reg_info)
		sum_write(str, me->reg_info);

	archive_strappend_char(str, '\n');
	if (mtree->indent || mtree->classic)
		mtree_indent(mtree);

	if (mtree->buf.length > 32768) {
		ret = __archive_write_output(
			a, mtree->buf.s, mtree->buf.length);
		archive_string_empty(&mtree->buf);
	} else
		ret = ARCHIVE_OK;
	return (ret);
}

static int
write_dot_dot_entry(struct archive_write *a, struct mtree_entry *n)
{
	struct mtree_writer *mtree = a->format_data;
	int ret;

	if (n->parentdir.s) {
		if (mtree->indent) {
			int i, pd = mtree->depth * 4;
			for (i = 0; i < pd; i++)
				archive_strappend_char(&mtree->buf, ' ');
		}
		archive_string_sprintf(&mtree->buf, "# %s/%s\n",
			n->parentdir.s, n->basename.s);
	}

	if (mtree->indent) {
		archive_string_empty(&mtree->ebuf);
		archive_strncat(&mtree->ebuf, "..\n\n", (mtree->dironly)?3:4);
		mtree_indent(mtree);
	} else
		archive_strncat(&mtree->buf, "..\n\n", (mtree->dironly)?3:4);

	if (mtree->buf.length > 32768) {
		ret = __archive_write_output(
			a, mtree->buf.s, mtree->buf.length);
		archive_string_empty(&mtree->buf);
	} else
		ret = ARCHIVE_OK;
	return (ret);
}

/*
 * Write mtree entries saved at attr_counter_set_collect() function.
 */
static int
write_mtree_entry_tree(struct archive_write *a)
{
	struct mtree_writer *mtree = a->format_data;
	struct mtree_entry *np = mtree->root;
	struct archive_rb_node *n;
	int ret;

	do {
		if (mtree->output_global_set) {
			/*
			 * Collect attribute information to know which value
			 * is frequently used among the children.
			 */
			attr_counter_set_reset(mtree);
			ARCHIVE_RB_TREE_FOREACH(n, &(np->dir_info->rbtree)) {
				struct mtree_entry *e = (struct mtree
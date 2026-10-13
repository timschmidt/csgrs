
/home/tim/Documents/GitHub/workspace/csgrs/target/outlined-same-radicand-products/cost-v2-binaries/outlined-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cd760 <<hyperreal::computable::node::RadicalCoefficient>::multiply>:
   cd760:	55                   	push   %rbp
   cd761:	41 57                	push   %r15
   cd763:	41 56                	push   %r14
   cd765:	41 55                	push   %r13
   cd767:	41 54                	push   %r12
   cd769:	53                   	push   %rbx
   cd76a:	48 81 ec 08 01 00 00 	sub    $0x108,%rsp
   cd771:	49 89 d5             	mov    %rdx,%r13
   cd774:	48 8d 5e 18          	lea    0x18(%rsi),%rbx
   cd778:	48 89 34 24          	mov    %rsi,(%rsp)
   cd77c:	48 83 7e 28 00       	cmpq   $0x0,0x28(%rsi)
   cd781:	74 30                	je     cd7b3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x53>
   cd783:	49 83 7d 10 00       	cmpq   $0x0,0x10(%r13)
   cd788:	75 41                	jne    cd7cb <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6b>
   cd78a:	49 83 7d 28 00       	cmpq   $0x0,0x28(%r13)
   cd78f:	75 3a                	jne    cd7cb <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6b>
   cd791:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
   cd796:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   cd79b:	4d 8d 65 08          	lea    0x8(%r13),%r12
   cd79f:	b0 01                	mov    $0x1,%al
   cd7a1:	89 44 24 0c          	mov    %eax,0xc(%rsp)
   cd7a5:	48 c7 44 24 48 00 00 	movq   $0x0,0x48(%rsp)
   cd7ac:	00 00 
   cd7ae:	e9 af 01 00 00       	jmp    cd962 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x202>
   cd7b3:	49 8b 45 28          	mov    0x28(%r13),%rax
   cd7b7:	48 85 c0             	test   %rax,%rax
   cd7ba:	74 37                	je     cd7f3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x93>
   cd7bc:	48 8b 0c 24          	mov    (%rsp),%rcx
   cd7c0:	48 83 79 10 00       	cmpq   $0x0,0x10(%rcx)
   cd7c5:	0f 84 b4 00 00 00    	je     cd87f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x11f>
   cd7cb:	48 c7 07 ff ff ff ff 	movq   $0xffffffffffffffff,(%rdi)
   cd7d2:	4c 8b 34 24          	mov    (%rsp),%r14
   cd7d6:	49 8b 46 30          	mov    0x30(%r14),%rax
   cd7da:	f0 48 ff 08          	lock decq (%rax)
   cd7de:	0f 85 10 03 00 00    	jne    cdaf4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x394>
   cd7e4:	49 8d 7e 30          	lea    0x30(%r14),%rdi
   cd7e8:	ff 15 f2 1b 0a 00    	call   *0xa1bf2(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd7ee:	e9 01 03 00 00       	jmp    cdaf4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x394>
   cd7f3:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
   cd7f8:	49 8b 4d 10          	mov    0x10(%r13),%rcx
   cd7fc:	48 8b 14 24          	mov    (%rsp),%rdx
   cd800:	48 8b 42 10          	mov    0x10(%rdx),%rax
   cd804:	48 83 f0 01          	xor    $0x1,%rax
   cd808:	4d 8d 65 08          	lea    0x8(%r13),%r12
   cd80c:	48 89 4c 24 48       	mov    %rcx,0x48(%rsp)
   cd811:	48 83 f1 01          	xor    $0x1,%rcx
   cd815:	48 09 c1             	or     %rax,%rcx
   cd818:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   cd81d:	0f 85 39 01 00 00    	jne    cd95c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1fc>
   cd823:	4c 8b 7a 08          	mov    0x8(%rdx),%r15
   cd827:	49 8b 34 24          	mov    (%r12),%rsi
   cd82b:	40 b5 01             	mov    $0x1,%bpl
   cd82e:	4c 89 ff             	mov    %r15,%rdi
   cd831:	ff 15 51 1b 0a 00    	call   *0xa1b51(%rip)        # 16f388 <_DYNAMIC+0x248>
   cd837:	b9 01 00 00 00       	mov    $0x1,%ecx
   cd83c:	48 89 4c 24 48       	mov    %rcx,0x48(%rsp)
   cd841:	84 c0                	test   %al,%al
   cd843:	0f 84 13 01 00 00    	je     cd95c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1fc>
   cd849:	49 8b 1f             	mov    (%r15),%rbx
   cd84c:	eb 0b                	jmp    cd859 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf9>
   cd84e:	66 90                	xchg   %ax,%ax
   cd850:	4c 89 fb             	mov    %r15,%rbx
   cd853:	49 89 df             	mov    %rbx,%r15
   cd856:	48 8b 1b             	mov    (%rbx),%rbx
   cd859:	8b 43 60             	mov    0x60(%rbx),%eax
   cd85c:	84 c0                	test   %al,%al
   cd85e:	79 50                	jns    cd8b0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x150>
   cd860:	8b 43 60             	mov    0x60(%rbx),%eax
   cd863:	84 c0                	test   %al,%al
   cd865:	79 e9                	jns    cd850 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0>
   cd867:	8b 43 50             	mov    0x50(%rbx),%eax
   cd86a:	48 83 c3 40          	add    $0x40,%rbx
   cd86e:	85 c0                	test   %eax,%eax
   cd870:	74 e1                	je     cd853 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf3>
   cd872:	48 89 df             	mov    %rbx,%rdi
   cd875:	4c 89 fe             	mov    %r15,%rsi
   cd878:	e8 2c 06 04 00       	call   10dea9 <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
   cd87d:	eb d4                	jmp    cd853 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf3>
   cd87f:	48 89 7c 24 10       	mov    %rdi,0x10(%rsp)
   cd884:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   cd889:	4d 8d 65 08          	lea    0x8(%r13),%r12
   cd88d:	49 8b 4d 10          	mov    0x10(%r13),%rcx
   cd891:	48 89 4c 24 48       	mov    %rcx,0x48(%rsp)
   cd896:	48 c1 e0 03          	shl    $0x3,%rax
   cd89a:	48 8d 04 40          	lea    (%rax,%rax,2),%rax
   cd89e:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cd8a3:	c7 44 24 0c 00 00 00 	movl   $0x0,0xc(%rsp)
   cd8aa:	00 
   cd8ab:	e9 bb 00 00 00       	jmp    cd96b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x20b>
   cd8b0:	80 7b 64 02          	cmpb   $0x2,0x64(%rbx)
   cd8b4:	75 5f                	jne    cd915 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1b5>
   cd8b6:	4c 8b 35 0b 1b 0a 00 	mov    0xa1b0b(%rip),%r14        # 16f3c8 <_DYNAMIC+0x288>
   cd8bd:	41 8b 46 18          	mov    0x18(%r14),%eax
   cd8c1:	85 c0                	test   %eax,%eax
   cd8c3:	0f 85 39 06 00 00    	jne    cdf02 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7a2>
   cd8c9:	48 8b 53 20          	mov    0x20(%rbx),%rdx
   cd8cd:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cd8d1:	75 42                	jne    cd915 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1b5>
   cd8d3:	49 8b 76 08          	mov    0x8(%r14),%rsi
   cd8d7:	48 8b 7b 18          	mov    0x18(%rbx),%rdi
   cd8db:	48 c1 e2 03          	shl    $0x3,%rdx
   cd8df:	ff 15 eb 1a 0a 00    	call   *0xa1aeb(%rip)        # 16f3d0 <bcmp@GLIBC_2.2.5>
   cd8e5:	85 c0                	test   %eax,%eax
   cd8e7:	75 2c                	jne    cd915 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1b5>
   cd8e9:	41 8b 46 18          	mov    0x18(%r14),%eax
   cd8ed:	85 c0                	test   %eax,%eax
   cd8ef:	0f 85 48 06 00 00    	jne    cdf3d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7dd>
   cd8f5:	48 8b 53 38          	mov    0x38(%rbx),%rdx
   cd8f9:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cd8fd:	75 16                	jne    cd915 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1b5>
   cd8ff:	49 8b 76 08          	mov    0x8(%r14),%rsi
   cd903:	48 8b 7b 30          	mov    0x30(%rbx),%rdi
   cd907:	48 c1 e2 03          	shl    $0x3,%rdx
   cd90b:	ff 15 bf 1a 0a 00    	call   *0xa1abf(%rip)        # 16f3d0 <bcmp@GLIBC_2.2.5>
   cd911:	85 c0                	test   %eax,%eax
   cd913:	74 47                	je     cd95c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1fc>
   cd915:	48 8b 0c 24          	mov    (%rsp),%rcx
   cd919:	48 8b 41 30          	mov    0x30(%rcx),%rax
   cd91d:	48 89 84 24 a0 00 00 	mov    %rax,0xa0(%rsp)
   cd924:	00 
   cd925:	0f 10 01             	movups (%rcx),%xmm0
   cd928:	0f 10 49 10          	movups 0x10(%rcx),%xmm1
   cd92c:	0f 10 51 20          	movups 0x20(%rcx),%xmm2
   cd930:	0f 29 94 24 90 00 00 	movaps %xmm2,0x90(%rsp)
   cd937:	00 
   cd938:	0f 29 8c 24 80 00 00 	movaps %xmm1,0x80(%rsp)
   cd93f:	00 
   cd940:	0f 29 44 24 70       	movaps %xmm0,0x70(%rsp)
   cd945:	48 8d 74 24 70       	lea    0x70(%rsp),%rsi
   cd94a:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
   cd94f:	4c 89 ea             	mov    %r13,%rdx
   cd952:	e8 89 f4 ff ff       	call   ccde0 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand>
   cd957:	e9 0e 02 00 00       	jmp    cdb6a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x40a>
   cd95c:	b0 01                	mov    $0x1,%al
   cd95e:	89 44 24 0c          	mov    %eax,0xc(%rsp)
   cd962:	48 c7 44 24 20 00 00 	movq   $0x0,0x20(%rsp)
   cd969:	00 00 
   cd96b:	48 8b 04 24          	mov    (%rsp),%rax
   cd96f:	48 8d 78 30          	lea    0x30(%rax),%rdi
   cd973:	49 8d 75 30          	lea    0x30(%r13),%rsi
   cd977:	40 b5 01             	mov    $0x1,%bpl
   cd97a:	48 89 bc 24 c0 00 00 	mov    %rdi,0xc0(%rsp)
   cd981:	00 
   cd982:	48 89 b4 24 d0 00 00 	mov    %rsi,0xd0(%rsp)
   cd989:	00 
   cd98a:	e8 71 3e fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd98f:	4c 89 ac 24 e0 00 00 	mov    %r13,0xe0(%rsp)
   cd996:	00 
   cd997:	48 89 84 24 a0 00 00 	mov    %rax,0xa0(%rsp)
   cd99e:	00 
   cd99f:	48 c7 44 24 70 00 00 	movq   $0x0,0x70(%rsp)
   cd9a6:	00 00 
   cd9a8:	48 c7 44 24 78 08 00 	movq   $0x8,0x78(%rsp)
   cd9af:	00 00 
   cd9b1:	0f 57 c0             	xorps  %xmm0,%xmm0
   cd9b4:	0f 11 84 24 80 00 00 	movups %xmm0,0x80(%rsp)
   cd9bb:	00 
   cd9bc:	48 c7 84 24 90 00 00 	movq   $0x8,0x90(%rsp)
   cd9c3:	00 08 00 00 00 
   cd9c8:	48 c7 84 24 98 00 00 	movq   $0x0,0x98(%rsp)
   cd9cf:	00 00 00 00 00 
   cd9d4:	48 8b 04 24          	mov    (%rsp),%rax
   cd9d8:	4c 8b 70 08          	mov    0x8(%rax),%r14
   cd9dc:	4c 8b 68 10          	mov    0x10(%rax),%r13
   cd9e0:	49 c1 e5 04          	shl    $0x4,%r13
   cd9e4:	48 8d 5c 24 70       	lea    0x70(%rsp),%rbx
   cd9e9:	48 8b ac 24 c0 00 00 	mov    0xc0(%rsp),%rbp
   cd9f0:	00 
   cd9f1:	66 66 66 66 66 66 2e 	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd9f8:	0f 1f 84 00 00 00 00 
   cd9ff:	00 
   cda00:	4d 85 ed             	test   %r13,%r13
   cda03:	74 42                	je     cda47 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2e7>
   cda05:	49 8b 06             	mov    (%r14),%rax
   cda08:	f0 48 ff 00          	lock incq (%rax)
   cda0c:	0f 8e 29 05 00 00    	jle    cdf3b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7db>
   cda12:	49 8d 7e 08          	lea    0x8(%r14),%rdi
   cda16:	4d 8b 3e             	mov    (%r14),%r15
   cda19:	4c 89 7c 24 28       	mov    %r15,0x28(%rsp)
   cda1e:	48 8b b4 24 d0 00 00 	mov    0xd0(%rsp),%rsi
   cda25:	00 
   cda26:	e8 d5 3d fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cda2b:	48 89 df             	mov    %rbx,%rdi
   cda2e:	4c 89 fe             	mov    %r15,%rsi
   cda31:	48 89 c2             	mov    %rax,%rdx
   cda34:	e8 c7 ee ff ff       	call   cc900 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cda39:	49 83 c6 10          	add    $0x10,%r14
   cda3d:	49 83 c5 f0          	add    $0xfffffffffffffff0,%r13
   cda41:	84 c0                	test   %al,%al
   cda43:	75 bb                	jne    cda00 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2a0>
   cda45:	eb 70                	jmp    cdab7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x357>
   cda47:	4d 8b 24 24          	mov    (%r12),%r12
   cda4b:	4c 8b 74 24 48       	mov    0x48(%rsp),%r14
   cda50:	49 c1 e6 04          	shl    $0x4,%r14
   cda54:	48 8d 5c 24 70       	lea    0x70(%rsp),%rbx
   cda59:	4c 89 b4 24 e8 00 00 	mov    %r14,0xe8(%rsp)
   cda60:	00 
   cda61:	4c 89 a4 24 f0 00 00 	mov    %r12,0xf0(%rsp)
   cda68:	00 
   cda69:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
   cda70:	4d 85 f6             	test   %r14,%r14
   cda73:	0f 84 03 01 00 00    	je     cdb7c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x41c>
   cda79:	49 8b 04 24          	mov    (%r12),%rax
   cda7d:	f0 48 ff 00          	lock incq (%rax)
   cda81:	0f 8e b4 04 00 00    	jle    cdf3b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7db>
   cda87:	49 8d 7c 24 08       	lea    0x8(%r12),%rdi
   cda8c:	4d 8b 3c 24          	mov    (%r12),%r15
   cda90:	4c 89 7c 24 28       	mov    %r15,0x28(%rsp)
   cda95:	48 89 ee             	mov    %rbp,%rsi
   cda98:	e8 63 3d fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cda9d:	48 89 df             	mov    %rbx,%rdi
   cdaa0:	4c 89 fe             	mov    %r15,%rsi
   cdaa3:	48 89 c2             	mov    %rax,%rdx
   cdaa6:	e8 55 ee ff ff       	call   cc900 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cdaab:	49 83 c4 10          	add    $0x10,%r12
   cdaaf:	49 83 c6 f0          	add    $0xfffffffffffffff0,%r14
   cdab3:	84 c0                	test   %al,%al
   cdab5:	75 b9                	jne    cda70 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x310>
   cdab7:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cdabc:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cdac3:	40 b5 01             	mov    $0x1,%bpl
   cdac6:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdacb:	e8 30 33 fe ff       	call   b0e00 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cdad0:	48 8b bc 24 c0 00 00 	mov    0xc0(%rsp),%rdi
   cdad7:	00 
   cdad8:	48 8b 07             	mov    (%rdi),%rax
   cdadb:	f0 48 ff 08          	lock decq (%rax)
   cdadf:	75 06                	jne    cdae7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x387>
   cdae1:	ff 15 f9 18 0a 00    	call   *0xa18f9(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdae7:	40 84 ed             	test   %bpl,%bpl
   cdaea:	4c 8b 34 24          	mov    (%rsp),%r14
   cdaee:	49 8d 5e 18          	lea    0x18(%r14),%rbx
   cdaf2:	74 23                	je     cdb17 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3b7>
   cdaf4:	4c 89 f7             	mov    %r14,%rdi
   cdaf7:	e8 34 59 04 00       	call   113430 <<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cdafc:	49 8b 36             	mov    (%r14),%rsi
   cdaff:	48 85 f6             	test   %rsi,%rsi
   cdb02:	74 13                	je     cdb17 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3b7>
   cdb04:	49 8b 7e 08          	mov    0x8(%r14),%rdi
   cdb08:	48 c1 e6 04          	shl    $0x4,%rsi
   cdb0c:	ba 08 00 00 00       	mov    $0x8,%edx
   cdb11:	ff 15 e9 18 0a 00    	call   *0xa18e9(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   cdb17:	49 8b 7e 20          	mov    0x20(%r14),%rdi
   cdb1b:	4d 8b 76 28          	mov    0x28(%r14),%r14
   cdb1f:	49 ff c6             	inc    %r14
   cdb22:	66 66 66 66 66 2e 0f 	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cdb29:	1f 84 00 00 00 00 00 
   cdb30:	49 83 fe 01          	cmp    $0x1,%r14
   cdb34:	74 11                	je     cdb47 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3e7>
   cdb36:	4c 8d 7f 18          	lea    0x18(%rdi),%r15
   cdb3a:	49 ff ce             	dec    %r14
   cdb3d:	e8 7e 3d 04 00       	call   1118c0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cdb42:	4c 89 ff             	mov    %r15,%rdi
   cdb45:	eb e9                	jmp    cdb30 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3d0>
   cdb47:	48 8b 03             	mov    (%rbx),%rax
   cdb4a:	48 85 c0             	test   %rax,%rax
   cdb4d:	74 1b                	je     cdb6a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x40a>
   cdb4f:	48 8b 0c 24          	mov    (%rsp),%rcx
   cdb53:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cdb57:	48 c1 e0 03          	shl    $0x3,%rax
   cdb5b:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cdb5f:	ba 08 00 00 00       	mov    $0x8,%edx
   cdb64:	ff 15 96 18 0a 00    	call   *0xa1896(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   cdb6a:	48 81 c4 08 01 00 00 	add    $0x108,%rsp
   cdb71:	5b                   	pop    %rbx
   cdb72:	41 5c                	pop    %r12
   cdb74:	41 5d                	pop    %r13
   cdb76:	41 5e                	pop    %r14
   cdb78:	41 5f                	pop    %r15
   cdb7a:	5d                   	pop    %rbp
   cdb7b:	c3                   	ret
   cdb7c:	48 8b 0c 24          	mov    (%rsp),%rcx
   cdb80:	48 8b 01             	mov    (%rcx),%rax
   cdb83:	48 8b 59 08          	mov    0x8(%rcx),%rbx
   cdb87:	48 8b 49 10          	mov    0x10(%rcx),%rcx
   cdb8b:	48 89 ca             	mov    %rcx,%rdx
   cdb8e:	48 c1 e2 04          	shl    $0x4,%rdx
   cdb92:	48 01 da             	add    %rbx,%rdx
   cdb95:	48 89 5c 24 28       	mov    %rbx,0x28(%rsp)
   cdb9a:	48 89 5c 24 30       	mov    %rbx,0x30(%rsp)
   cdb9f:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cdba4:	48 89 94 24 d8 00 00 	mov    %rdx,0xd8(%rsp)
   cdbab:	00 
   cdbac:	48 89 54 24 40       	mov    %rdx,0x40(%rsp)
   cdbb1:	48 85 c9             	test   %rcx,%rcx
   cdbb4:	0f 84 b9 01 00 00    	je     cdd73 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x613>
   cdbba:	4c 8d ac 24 c8 00 00 	lea    0xc8(%rsp),%r13
   cdbc1:	00 
   cdbc2:	eb 0e                	jmp    cdbd2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x472>
   cdbc4:	48 3b 9c 24 d8 00 00 	cmp    0xd8(%rsp),%rbx
   cdbcb:	00 
   cdbcc:	0f 84 a1 01 00 00    	je     cdd73 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x613>
   cdbd2:	48 8d 4b 10          	lea    0x10(%rbx),%rcx
   cdbd6:	48 89 8c 24 f8 00 00 	mov    %rcx,0xf8(%rsp)
   cdbdd:	00 
   cdbde:	48 89 4c 24 30       	mov    %rcx,0x30(%rsp)
   cdbe3:	48 8b 0b             	mov    (%rbx),%rcx
   cdbe6:	48 8b 43 08          	mov    0x8(%rbx),%rax
   cdbea:	48 89 8c 24 b0 00 00 	mov    %rcx,0xb0(%rsp)
   cdbf1:	00 
   cdbf2:	48 89 8c 24 c8 00 00 	mov    %rcx,0xc8(%rsp)
   cdbf9:	00 
   cdbfa:	48 89 84 24 b8 00 00 	mov    %rax,0xb8(%rsp)
   cdc01:	00 
   cdc02:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
   cdc07:	4c 8b b4 24 e8 00 00 	mov    0xe8(%rsp),%r14
   cdc0e:	00 
   cdc0f:	48 8b ac 24 f0 00 00 	mov    0xf0(%rsp),%rbp
   cdc16:	00 
   cdc17:	48 83 7c 24 48 00    	cmpq   $0x0,0x48(%rsp)
   cdc1d:	75 5e                	jne    cdc7d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x51d>
   cdc1f:	48 8b 84 24 b8 00 00 	mov    0xb8(%rsp),%rax
   cdc26:	00 
   cdc27:	f0 48 ff 08          	lock decq (%rax)
   cdc2b:	75 0b                	jne    cdc38 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4d8>
   cdc2d:	48 8d 7c 24 58       	lea    0x58(%rsp),%rdi
   cdc32:	ff 15 a8 17 0a 00    	call   *0xa17a8(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdc38:	48 8b 84 24 b0 00 00 	mov    0xb0(%rsp),%rax
   cdc3f:	00 
   cdc40:	f0 48 ff 08          	lock decq (%rax)
   cdc44:	48 8b 9c 24 f8 00 00 	mov    0xf8(%rsp),%rbx
   cdc4b:	00 
   cdc4c:	0f 85 72 ff ff ff    	jne    cdbc4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x464>
   cdc52:	4c 89 ef             	mov    %r13,%rdi
   cdc55:	ff 15 85 17 0a 00    	call   *0xa1785(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdc5b:	e9 64 ff ff ff       	jmp    cdbc4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x464>
   cdc60:	4c 89 ff             	mov    %r15,%rdi
   cdc63:	48 89 de             	mov    %rbx,%rsi
   cdc66:	e8 65 22 fc ff       	call   8fed0 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cdc6b:	48 89 84 24 a0 00 00 	mov    %rax,0xa0(%rsp)
   cdc72:	00 
   cdc73:	48 83 c5 10          	add    $0x10,%rbp
   cdc77:	49 83 c6 f0          	add    $0xfffffffffffffff0,%r14
   cdc7b:	74 a2                	je     cdc1f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4bf>
   cdc7d:	48 8d 75 08          	lea    0x8(%rbp),%rsi
   cdc81:	48 8d 7c 24 58       	lea    0x58(%rsp),%rdi
   cdc86:	e8 75 3b fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cdc8b:	49 89 c4             	mov    %rax,%r12
   cdc8e:	48 89 84 24 00 01 00 	mov    %rax,0x100(%rsp)
   cdc95:	00 
   cdc96:	4c 89 ef             	mov    %r13,%rdi
   cdc99:	48 89 ee             	mov    %rbp,%rsi
   cdc9c:	ff 15 e6 16 0a 00    	call   *0xa16e6(%rip)        # 16f388 <_DYNAMIC+0x248>
   cdca2:	84 c0                	test   %al,%al
   cdca4:	74 3a                	je     cdce0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x580>
   cdca6:	4c 8b bc 24 a0 00 00 	mov    0xa0(%rsp),%r15
   cdcad:	00 
   cdcae:	4c 89 7c 24 50       	mov    %r15,0x50(%rsp)
   cdcb3:	4c 89 64 24 60       	mov    %r12,0x60(%rsp)
   cdcb8:	48 8d 7c 24 60       	lea    0x60(%rsp),%rdi
   cdcbd:	4c 89 ee             	mov    %r13,%rsi
   cdcc0:	e8 3b 3b fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cdcc5:	48 89 c3             	mov    %rax,%rbx
   cdcc8:	f0 49 ff 0c 24       	lock decq (%r12)
   cdccd:	75 91                	jne    cdc60 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x500>
   cdccf:	48 8d 7c 24 60       	lea    0x60(%rsp),%rdi
   cdcd4:	ff 15 06 17 0a 00    	call   *0xa1706(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdcda:	eb 84                	jmp    cdc60 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x500>
   cdcdc:	0f 1f 40 00          	nopl   0x0(%rax)
   cdce0:	4c 89 ef             	mov    %r13,%rdi
   cdce3:	48 89 ee             	mov    %rbp,%rsi
   cdce6:	e8 15 3b fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cdceb:	48 89 c7             	mov    %rax,%rdi
   cdcee:	e8 dd 56 fa ff       	call   733d0 <<hyperreal::rational::arithmetic::Rational>::extract_square_reduced_retained>
   cdcf3:	49 89 d7             	mov    %rdx,%r15
   cdcf6:	48 89 54 24 60       	mov    %rdx,0x60(%rsp)
   cdcfb:	4c 89 e7             	mov    %r12,%rdi
   cdcfe:	48 89 c6             	mov    %rax,%rsi
   cdd01:	e8 7a 46 fc ff       	call   92380 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cdd06:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdd0b:	4c 89 fe             	mov    %r15,%rsi
   cdd0e:	48 89 c2             	mov    %rax,%rdx
   cdd11:	e8 ea eb ff ff       	call   cc900 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cdd16:	84 c0                	test   %al,%al
   cdd18:	0f 85 55 ff ff ff    	jne    cdc73 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x513>
   cdd1e:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cdd23:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cdd2a:	48 8b 84 24 b8 00 00 	mov    0xb8(%rsp),%rax
   cdd31:	00 
   cdd32:	f0 48 ff 08          	lock decq (%rax)
   cdd36:	75 0b                	jne    cdd43 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5e3>
   cdd38:	48 8d 7c 24 58       	lea    0x58(%rsp),%rdi
   cdd3d:	ff 15 9d 16 0a 00    	call   *0xa169d(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdd43:	48 8b 84 24 b0 00 00 	mov    0xb0(%rsp),%rax
   cdd4a:	00 
   cdd4b:	f0 48 ff 08          	lock decq (%rax)
   cdd4f:	75 0e                	jne    cdd5f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5ff>
   cdd51:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cdd58:	00 
   cdd59:	ff 15 81 16 0a 00    	call   *0xa1681(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdd5f:	41 b7 01             	mov    $0x1,%r15b
   cdd62:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cdd67:	e8 54 fc 03 00       	call   10d9c0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cdd6c:	31 ed                	xor    %ebp,%ebp
   cdd6e:	e9 53 fd ff ff       	jmp    cdac6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x366>
   cdd73:	41 b7 01             	mov    $0x1,%r15b
   cdd76:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cdd7b:	e8 40 fc 03 00       	call   10d9c0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cdd80:	48 8b 0c 24          	mov    (%rsp),%rcx
   cdd84:	48 8b 41 18          	mov    0x18(%rcx),%rax
   cdd88:	4c 8b 71 20          	mov    0x20(%rcx),%r14
   cdd8c:	48 8b 49 28          	mov    0x28(%rcx),%rcx
   cdd90:	48 8d 14 cd 00 00 00 	lea    0x0(,%rcx,8),%rdx
   cdd97:	00 
   cdd98:	4c 8d 24 52          	lea    (%rdx,%rdx,2),%r12
   cdd9c:	4b 8d 14 26          	lea    (%r14,%r12,1),%rdx
   cdda0:	4c 89 74 24 28       	mov    %r14,0x28(%rsp)
   cdda5:	4c 89 74 24 30       	mov    %r14,0x30(%rsp)
   cddaa:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cddaf:	48 89 54 24 40       	mov    %rdx,0x40(%rsp)
   cddb4:	48 85 c9             	test   %rcx,%rcx
   cddb7:	74 6f                	je     cde28 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6c8>
   cddb9:	49 83 c6 18          	add    $0x18,%r14
   cddbd:	0f 1f 00             	nopl   (%rax)
   cddc0:	4c 89 74 24 30       	mov    %r14,0x30(%rsp)
   cddc5:	4d 8b 7e e8          	mov    -0x18(%r14),%r15
   cddc9:	4d 85 ff             	test   %r15,%r15
   cddcc:	74 5a                	je     cde28 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6c8>
   cddce:	4d 8b 6e f0          	mov    -0x10(%r14),%r13
   cddd2:	49 8b 5e f8          	mov    -0x8(%r14),%rbx
   cddd6:	4c 89 7c 24 60       	mov    %r15,0x60(%rsp)
   cdddb:	4c 89 6c 24 68       	mov    %r13,0x68(%rsp)
   cdde0:	48 89 5c 24 50       	mov    %rbx,0x50(%rsp)
   cdde5:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cddea:	48 8b b4 24 d0 00 00 	mov    0xd0(%rsp),%rsi
   cddf1:	00 
   cddf2:	e8 09 3a fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cddf7:	48 89 c5             	mov    %rax,%rbp
   cddfa:	f0 48 ff 0b          	lock decq (%rbx)
   cddfe:	75 0b                	jne    cde0b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6ab>
   cde00:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cde05:	ff 15 d5 15 0a 00    	call   *0xa15d5(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cde0b:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cde10:	4c 89 fe             	mov    %r15,%rsi
   cde13:	4c 89 ea             	mov    %r13,%rdx
   cde16:	48 89 e9             	mov    %rbp,%rcx
   cde19:	e8 92 e7 ff ff       	call   cc5b0 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cde1e:	49 83 c6 18          	add    $0x18,%r14
   cde22:	49 83 c4 e8          	add    $0xffffffffffffffe8,%r12
   cde26:	75 98                	jne    cddc0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x660>
   cde28:	45 31 ff             	xor    %r15d,%r15d
   cde2b:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cde30:	e8 bb fa 03 00       	call   10d8f0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cde35:	80 7c 24 0c 00       	cmpb   $0x0,0xc(%rsp)
   cde3a:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
   cde3f:	48 8b ac 24 c0 00 00 	mov    0xc0(%rsp),%rbp
   cde46:	00 
   cde47:	75 6e                	jne    cdeb7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x757>
   cde49:	48 8b 84 24 e0 00 00 	mov    0xe0(%rsp),%rax
   cde50:	00 
   cde51:	48 8b 58 20          	mov    0x20(%rax),%rbx
   cde55:	48 01 5c 24 20       	add    %rbx,0x20(%rsp)
   cde5a:	4c 8d 7c 24 70       	lea    0x70(%rsp),%r15
   cde5f:	90                   	nop
   cde60:	4c 8b 23             	mov    (%rbx),%r12
   cde63:	4c 8b 6b 08          	mov    0x8(%rbx),%r13
   cde67:	f0 49 ff 04 24       	lock incq (%r12)
   cde6c:	0f 8e c9 00 00 00    	jle    cdf3b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7db>
   cde72:	4d 85 ed             	test   %r13,%r13
   cde75:	74 0b                	je     cde82 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x722>
   cde77:	f0 49 ff 45 00       	lock incq 0x0(%r13)
   cde7c:	0f 8e b9 00 00 00    	jle    cdf3b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7db>
   cde82:	48 83 c3 10          	add    $0x10,%rbx
   cde86:	4c 89 64 24 28       	mov    %r12,0x28(%rsp)
   cde8b:	4c 89 6c 24 30       	mov    %r13,0x30(%rsp)
   cde90:	48 89 df             	mov    %rbx,%rdi
   cde93:	48 89 ee             	mov    %rbp,%rsi
   cde96:	e8 65 39 fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cde9b:	4c 89 ff             	mov    %r15,%rdi
   cde9e:	4c 89 e6             	mov    %r12,%rsi
   cdea1:	4c 89 ea             	mov    %r13,%rdx
   cdea4:	48 89 c1             	mov    %rax,%rcx
   cdea7:	e8 04 e7 ff ff       	call   cc5b0 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cdeac:	48 83 c3 08          	add    $0x8,%rbx
   cdeb0:	48 3b 5c 24 20       	cmp    0x20(%rsp),%rbx
   cdeb5:	75 a9                	jne    cde60 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x700>
   cdeb7:	48 8b 84 24 a0 00 00 	mov    0xa0(%rsp),%rax
   cdebe:	00 
   cdebf:	49 89 46 30          	mov    %rax,0x30(%r14)
   cdec3:	0f 10 44 24 70       	movups 0x70(%rsp),%xmm0
   cdec8:	0f 10 8c 24 80 00 00 	movups 0x80(%rsp),%xmm1
   cdecf:	00 
   cded0:	0f 10 94 24 90 00 00 	movups 0x90(%rsp),%xmm2
   cded7:	00 
   cded8:	41 0f 11 56 20       	movups %xmm2,0x20(%r14)
   cdedd:	41 0f 11 4e 10       	movups %xmm1,0x10(%r14)
   cdee2:	41 0f 11 06          	movups %xmm0,(%r14)
   cdee6:	48 8b 45 00          	mov    0x0(%rbp),%rax
   cdeea:	f0 48 ff 08          	lock decq (%rax)
   cdeee:	0f 85 76 fc ff ff    	jne    cdb6a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x40a>
   cdef4:	48 89 ef             	mov    %rbp,%rdi
   cdef7:	ff 15 e3 14 0a 00    	call   *0xa14e3(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdefd:	e9 68 fc ff ff       	jmp    cdb6a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x40a>
   cdf02:	4c 89 74 24 28       	mov    %r14,0x28(%rsp)
   cdf07:	48 8d 44 24 28       	lea    0x28(%rsp),%rax
   cdf0c:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
   cdf11:	40 b5 01             	mov    $0x1,%bpl
   cdf14:	49 8d 7e 18          	lea    0x18(%r14),%rdi
   cdf18:	48 8d 0d 81 d1 09 00 	lea    0x9d181(%rip),%rcx        # 16b0a0 <anon.afeede234644f26776946e76b404b106.3.llvm.3222190057612512104+0x28>
   cdf1f:	4c 8d 05 3a d1 09 00 	lea    0x9d13a(%rip),%r8        # 16b060 <anon.afeede234644f26776946e76b404b106.2.llvm.3222190057612512104>
   cdf26:	48 8d 54 24 70       	lea    0x70(%rsp),%rdx
   cdf2b:	be 01 00 00 00       	mov    $0x1,%esi
   cdf30:	ff 15 4a 14 0a 00    	call   *0xa144a(%rip)        # 16f380 <_DYNAMIC+0x240>
   cdf36:	e9 8e f9 ff ff       	jmp    cd8c9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x169>
   cdf3b:	0f 0b                	ud2
   cdf3d:	4c 89 74 24 28       	mov    %r14,0x28(%rsp)
   cdf42:	48 8d 44 24 28       	lea    0x28(%rsp),%rax
   cdf47:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
   cdf4c:	40 b5 01             	mov    $0x1,%bpl
   cdf4f:	49 8d 7e 18          	lea    0x18(%r14),%rdi
   cdf53:	48 8d 0d 46 d1 09 00 	lea    0x9d146(%rip),%rcx        # 16b0a0 <anon.afeede234644f26776946e76b404b106.3.llvm.3222190057612512104+0x28>
   cdf5a:	4c 8d 05 ff d0 09 00 	lea    0x9d0ff(%rip),%r8        # 16b060 <anon.afeede234644f26776946e76b404b106.2.llvm.3222190057612512104>
   cdf61:	48 8d 54 24 70       	lea    0x70(%rsp),%rdx
   cdf66:	be 01 00 00 00       	mov    $0x1,%esi
   cdf6b:	ff 15 0f 14 0a 00    	call   *0xa140f(%rip)        # 16f380 <_DYNAMIC+0x240>
   cdf71:	e9 7f f9 ff ff       	jmp    cd8f5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x195>
   cdf76:	49 89 c5             	mov    %rax,%r13
   cdf79:	40 b5 01             	mov    $0x1,%bpl
   cdf7c:	e9 a5 00 00 00       	jmp    ce026 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8c6>
   cdf81:	eb 02                	jmp    cdf85 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x825>
   cdf83:	eb 0b                	jmp    cdf90 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x830>
   cdf85:	49 89 c5             	mov    %rax,%r13
   cdf88:	41 b6 01             	mov    $0x1,%r14b
   cdf8b:	e9 f3 01 00 00       	jmp    ce183 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa23>
   cdf90:	49 89 c5             	mov    %rax,%r13
   cdf93:	41 b6 01             	mov    $0x1,%r14b
   cdf96:	e9 cc 01 00 00       	jmp    ce167 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa07>
   cdf9b:	49 89 c5             	mov    %rax,%r13
   cdf9e:	31 ed                	xor    %ebp,%ebp
   cdfa0:	e9 70 01 00 00       	jmp    ce115 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b5>
   cdfa5:	49 89 c5             	mov    %rax,%r13
   cdfa8:	40 84 ed             	test   %bpl,%bpl
   cdfab:	75 4f                	jne    cdffc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x89c>
   cdfad:	e9 a8 00 00 00       	jmp    ce05a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8fa>
   cdfb2:	49 89 c5             	mov    %rax,%r13
   cdfb5:	eb 19                	jmp    cdfd0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x870>
   cdfb7:	49 89 c5             	mov    %rax,%r13
   cdfba:	eb 1e                	jmp    cdfda <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x87a>
   cdfbc:	49 89 c5             	mov    %rax,%r13
   cdfbf:	f0 48 ff 0b          	lock decq (%rbx)
   cdfc3:	75 0b                	jne    cdfd0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x870>
   cdfc5:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cdfca:	ff 15 10 14 0a 00    	call   *0xa1410(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cdfd0:	48 8d 7c 24 60       	lea    0x60(%rsp),%rdi
   cdfd5:	e8 86 29 fe ff       	call   b0960 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cdfda:	31 ed                	xor    %ebp,%ebp
   cdfdc:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cdfe1:	e8 0a f9 03 00       	call   10d8f0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cdfe6:	45 31 ff             	xor    %r15d,%r15d
   cdfe9:	e9 27 01 00 00       	jmp    ce115 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b5>
   cdfee:	ff 15 f4 13 0a 00    	call   *0xa13f4(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   cdff4:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   cdff9:	49 89 c5             	mov    %rax,%r13
   cdffc:	48 8b 3c 24          	mov    (%rsp),%rdi
   ce000:	e8 cb 1a fe ff       	call   afad0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   ce005:	eb 53                	jmp    ce05a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8fa>
   ce007:	49 89 c5             	mov    %rax,%r13
   ce00a:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   ce00f:	e8 4c 29 fe ff       	call   b0960 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   ce014:	eb 03                	jmp    ce019 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8b9>
   ce016:	49 89 c5             	mov    %rax,%r13
   ce019:	31 ed                	xor    %ebp,%ebp
   ce01b:	45 31 ff             	xor    %r15d,%r15d
   ce01e:	e9 f2 00 00 00       	jmp    ce115 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b5>
   ce023:	49 89 c5             	mov    %rax,%r13
   ce026:	41 b7 01             	mov    $0x1,%r15b
   ce029:	e9 84 01 00 00       	jmp    ce1b2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa52>
   ce02e:	49 89 c5             	mov    %rax,%r13
   ce031:	eb 70                	jmp    ce0a3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x943>
   ce033:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   ce038:	49 89 c5             	mov    %rax,%r13
   ce03b:	49 8b 36             	mov    (%r14),%rsi
   ce03e:	48 85 f6             	test   %rsi,%rsi
   ce041:	74 17                	je     ce05a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8fa>
   ce043:	48 8b 04 24          	mov    (%rsp),%rax
   ce047:	48 8b 78 08          	mov    0x8(%rax),%rdi
   ce04b:	48 c1 e6 04          	shl    $0x4,%rsi
   ce04f:	ba 08 00 00 00       	mov    $0x8,%edx
   ce054:	ff 15 a6 13 0a 00    	call   *0xa13a6(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   ce05a:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
   ce05f:	e8 9c 19 fe ff       	call   afa00 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   ce064:	e9 b6 01 00 00       	jmp    ce21f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xabf>
   ce069:	49 89 c5             	mov    %rax,%r13
   ce06c:	f0 49 ff 0f          	lock decq (%r15)
   ce070:	41 b6 01             	mov    $0x1,%r14b
   ce073:	0f 85 d5 00 00 00    	jne    ce14e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9ee>
   ce079:	48 8d 7c 24 60       	lea    0x60(%rsp),%rdi
   ce07e:	ff 15 5c 13 0a 00    	call   *0xa135c(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce084:	e9 c5 00 00 00       	jmp    ce14e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9ee>
   ce089:	49 89 c5             	mov    %rax,%r13
   ce08c:	eb 26                	jmp    ce0b4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x954>
   ce08e:	49 89 c5             	mov    %rax,%r13
   ce091:	f0 49 ff 0c 24       	lock decq (%r12)
   ce096:	75 0b                	jne    ce0a3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x943>
   ce098:	48 8d 7c 24 60       	lea    0x60(%rsp),%rdi
   ce09d:	ff 15 3d 13 0a 00    	call   *0xa133d(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce0a3:	f0 49 ff 0f          	lock decq (%r15)
   ce0a7:	75 0b                	jne    ce0b4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x954>
   ce0a9:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   ce0ae:	ff 15 2c 13 0a 00    	call   *0xa132c(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce0b4:	45 31 f6             	xor    %r14d,%r14d
   ce0b7:	e9 92 00 00 00       	jmp    ce14e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9ee>
   ce0bc:	ff 15 26 13 0a 00    	call   *0xa1326(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   ce0c2:	eb 21                	jmp    ce0e5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x985>
   ce0c4:	49 89 c5             	mov    %rax,%r13
   ce0c7:	f0 49 ff 0f          	lock decq (%r15)
   ce0cb:	40 b5 01             	mov    $0x1,%bpl
   ce0ce:	75 42                	jne    ce112 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b2>
   ce0d0:	40 b5 01             	mov    $0x1,%bpl
   ce0d3:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   ce0d8:	ff 15 02 13 0a 00    	call   *0xa1302(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce0de:	41 b7 01             	mov    $0x1,%r15b
   ce0e1:	eb 32                	jmp    ce115 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b5>
   ce0e3:	eb 27                	jmp    ce10c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9ac>
   ce0e5:	49 89 c5             	mov    %rax,%r13
   ce0e8:	41 b6 01             	mov    $0x1,%r14b
   ce0eb:	eb 61                	jmp    ce14e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9ee>
   ce0ed:	49 89 c5             	mov    %rax,%r13
   ce0f0:	f0 49 ff 0f          	lock decq (%r15)
   ce0f4:	40 b5 01             	mov    $0x1,%bpl
   ce0f7:	75 19                	jne    ce112 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b2>
   ce0f9:	40 b5 01             	mov    $0x1,%bpl
   ce0fc:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   ce101:	ff 15 d9 12 0a 00    	call   *0xa12d9(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce107:	41 b7 01             	mov    $0x1,%r15b
   ce10a:	eb 09                	jmp    ce115 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b5>
   ce10c:	49 89 c5             	mov    %rax,%r13
   ce10f:	40 b5 01             	mov    $0x1,%bpl
   ce112:	41 b7 01             	mov    $0x1,%r15b
   ce115:	48 8b 84 24 a0 00 00 	mov    0xa0(%rsp),%rax
   ce11c:	00 
   ce11d:	f0 48 ff 08          	lock decq (%rax)
   ce121:	75 78                	jne    ce19b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa3b>
   ce123:	48 8d bc 24 a0 00 00 	lea    0xa0(%rsp),%rdi
   ce12a:	00 
   ce12b:	ff 15 af 12 0a 00    	call   *0xa12af(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce131:	eb 68                	jmp    ce19b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa3b>
   ce133:	49 89 c5             	mov    %rax,%r13
   ce136:	f0 49 ff 0c 24       	lock decq (%r12)
   ce13b:	41 b6 01             	mov    $0x1,%r14b
   ce13e:	75 0e                	jne    ce14e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9ee>
   ce140:	48 8d bc 24 00 01 00 	lea    0x100(%rsp),%rdi
   ce147:	00 
   ce148:	ff 15 92 12 0a 00    	call   *0xa1292(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce14e:	48 8b 84 24 b8 00 00 	mov    0xb8(%rsp),%rax
   ce155:	00 
   ce156:	f0 48 ff 08          	lock decq (%rax)
   ce15a:	75 0b                	jne    ce167 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa07>
   ce15c:	48 8d 7c 24 58       	lea    0x58(%rsp),%rdi
   ce161:	ff 15 79 12 0a 00    	call   *0xa1279(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce167:	48 8b 84 24 b0 00 00 	mov    0xb0(%rsp),%rax
   ce16e:	00 
   ce16f:	f0 48 ff 08          	lock decq (%rax)
   ce173:	75 0e                	jne    ce183 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa23>
   ce175:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   ce17c:	00 
   ce17d:	ff 15 5d 12 0a 00    	call   *0xa125d(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce183:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   ce188:	e8 33 f8 03 00       	call   10d9c0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   ce18d:	41 b7 01             	mov    $0x1,%r15b
   ce190:	31 ed                	xor    %ebp,%ebp
   ce192:	45 84 f6             	test   %r14b,%r14b
   ce195:	0f 85 7a ff ff ff    	jne    ce115 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b5>
   ce19b:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   ce1a0:	e8 2b 19 fe ff       	call   afad0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   ce1a5:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   ce1ac:	00 
   ce1ad:	e8 4e 18 fe ff       	call   afa00 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   ce1b2:	48 8b 04 24          	mov    (%rsp),%rax
   ce1b6:	48 8b 40 30          	mov    0x30(%rax),%rax
   ce1ba:	f0 48 ff 08          	lock decq (%rax)
   ce1be:	75 0e                	jne    ce1ce <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa6e>
   ce1c0:	48 8b 04 24          	mov    (%rsp),%rax
   ce1c4:	48 8d 78 30          	lea    0x30(%rax),%rdi
   ce1c8:	ff 15 12 12 0a 00    	call   *0xa1212(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ce1ce:	40 84 ed             	test   %bpl,%bpl
   ce1d1:	74 09                	je     ce1dc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa7c>
   ce1d3:	48 8b 3c 24          	mov    (%rsp),%rdi
   ce1d7:	e8 f4 18 fe ff       	call   afad0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   ce1dc:	45 84 ff             	test   %r15b,%r15b
   ce1df:	74 3e                	je     ce21f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xabf>
   ce1e1:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
   ce1e6:	e8 15 18 fe ff       	call   afa00 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   ce1eb:	eb 32                	jmp    ce21f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xabf>
   ce1ed:	ff 15 f5 11 0a 00    	call   *0xa11f5(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   ce1f3:	49 89 dc             	mov    %rbx,%r12
   ce1f6:	49 89 c5             	mov    %rax,%r13
   ce1f9:	0f 1f 80 00 00 00 00 	nopl   0x0(%rax)
   ce200:	49 ff ce             	dec    %r14
   ce203:	74 11                	je     ce216 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xab6>
   ce205:	49 8d 5f 18          	lea    0x18(%r15),%rbx
   ce209:	4c 89 ff             	mov    %r15,%rdi
   ce20c:	e8 af 36 04 00       	call   1118c0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   ce211:	49 89 df             	mov    %rbx,%r15
   ce214:	eb ea                	jmp    ce200 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xaa0>
   ce216:	49 8b 04 24          	mov    (%r12),%rax
   ce21a:	48 85 c0             	test   %rax,%rax
   ce21d:	75 08                	jne    ce227 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xac7>
   ce21f:	4c 89 ef             	mov    %r13,%rdi
   ce222:	e8 59 ae 09 00       	call   169080 <_Unwind_Resume@plt>
   ce227:	48 8b 0c 24          	mov    (%rsp),%rcx
   ce22b:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   ce22f:	48 c1 e0 03          	shl    $0x3,%rax
   ce233:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   ce237:	ba 08 00 00 00       	mov    $0x8,%edx
   ce23c:	ff 15 be 11 0a 00    	call   *0xa11be(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   ce242:	4c 89 ef             	mov    %r13,%rdi
   ce245:	e8 36 ae 09 00       	call   169080 <_Unwind_Resume@plt>
   ce24a:	ff 15 98 11 0a 00    	call   *0xa1198(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>

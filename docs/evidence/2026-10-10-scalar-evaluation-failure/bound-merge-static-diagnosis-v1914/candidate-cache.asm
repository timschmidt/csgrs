
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1850-bound-merge-atan-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000bd300 <<hyperreal::computable::node::EvaluationCache>::at_precision>:
   bd300:	push   %rbp
   bd301:	push   %r15
   bd303:	push   %r14
   bd305:	push   %r13
   bd307:	push   %r12
   bd309:	push   %rbx
   bd30a:	sub    $0x68,%rsp
   bd30e:	mov    %rdi,%r14
   bd311:	mov    (%rsi),%rbx
   bd314:	test   %rbx,%rbx
   bd317:	je     bd357 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x57>
   bd319:	mov    (%rbx),%eax
   bd31b:	cmp    $0x3ffffffd,%eax
   bd320:	ja     bd655 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bd326:	lea    0x1(%rax),%ecx
   bd329:	lock cmpxchg %ecx,(%rbx)
   bd32d:	jne    bd655 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bd333:	movzbl 0x8(%rbx),%eax
   bd337:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bd33c:	je     bd673 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x373>
   bd342:	lea    0x10(%rbx),%r15
   bd346:	sub    0x30(%rbx),%edx
   bd349:	jge    bd363 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x63>
   bd34b:	movq   $0xffffffffffffffff,(%r14)
   bd352:	jmp    bd634 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   bd357:	movq   $0xffffffffffffffff,(%r14)
   bd35e:	jmp    bd690 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bd363:	jne    bd3d6 <<hyperreal::computable::node::EvaluationCache>::at_precision+0xd6>
   bd365:	mov    0x20(%rbx),%rbp
   bd369:	lea    0x0(,%rbp,8),%r15
   bd371:	mov    %rbp,%rax
   bd374:	shr    $0x3d,%rax
   bd378:	setne  %al
   bd37b:	movabs $0x7ffffffffffffff8,%rcx
   bd385:	cmp    %rcx,%r15
   bd388:	seta   %cl
   bd38b:	or     %al,%cl
   bd38d:	jne    bd405 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x105>
   bd38f:	movzbl 0x28(%rbx),%eax
   bd393:	mov    %al,0xc(%rsp)
   bd397:	mov    0x18(%rbx),%rsi
   bd39b:	test   %r15,%r15
   bd39e:	je     bd490 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x190>
   bd3a4:	mov    %rsi,0x58(%rsp)
   bd3a9:	call   *0xcc011(%rip)        # 1893c0 <_DYNAMIC+0x240>
   bd3af:	mov    $0x8,%esi
   bd3b4:	mov    %r15,%rdi
   bd3b7:	call   *0xcc00b(%rip)        # 1893c8 <_DYNAMIC+0x248>
   bd3bd:	test   %rax,%rax
   bd3c0:	je     bd6fc <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   bd3c6:	mov    %rax,%r13
   bd3c9:	mov    %rbp,%r12
   bd3cc:	mov    0x58(%rsp),%rsi
   bd3d1:	jmp    bd499 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x199>
   bd3d6:	cmp    $0x1,%edx
   bd3d9:	jne    bd412 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x112>
   bd3db:	mov    0x20(%rbx),%r12
   bd3df:	lea    0x0(,%r12,8),%r15
   bd3e7:	mov    %r12,%rax
   bd3ea:	shr    $0x3d,%rax
   bd3ee:	setne  %al
   bd3f1:	movabs $0x7ffffffffffffff8,%rcx
   bd3fb:	cmp    %rcx,%r15
   bd3fe:	seta   %cl
   bd401:	or     %al,%cl
   bd403:	je     bd45d <<hyperreal::computable::node::EvaluationCache>::at_precision+0x15d>
   bd405:	xor    %edi,%edi
   bd407:	mov    %r15,%rsi
   bd40a:	call   *0xcc018(%rip)        # 189428 <_DYNAMIC+0x2a8>
   bd410:	ud2
   bd412:	mov    0x20(%rbx),%rax
   bd416:	test   %rax,%rax
   bd419:	je     bd440 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x140>
   bd41b:	mov    0x18(%rbx),%rcx
   bd41f:	mov    %edx,%esi
   bd421:	mov    $0x7f,%edi
   bd426:	bsr    -0x8(%rcx,%rax,8),%rdi
   bd42c:	xor    $0x3f,%rdi
   bd430:	shl    $0x6,%rax
   bd434:	sub    %rdi,%rax
   bd437:	cmp    %rsi,%rax
   bd43a:	jae    bd4c3 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1c3>
   bd440:	movups -0x89e17(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387+0x10>
   bd447:	movaps %xmm0,0x20(%rsp)
   bd44c:	movups -0x89e33(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.2438011856832709387>
   bd453:	movaps %xmm0,0x10(%rsp)
   bd458:	jmp    bd621 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x321>
   bd45d:	movzbl 0x28(%rbx),%ebp
   bd461:	mov    0x18(%rbx),%r13
   bd465:	test   %r15,%r15
   bd468:	je     bd508 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x208>
   bd46e:	call   *0xcbf4c(%rip)        # 1893c0 <_DYNAMIC+0x240>
   bd474:	mov    $0x8,%esi
   bd479:	mov    %r15,%rdi
   bd47c:	call   *0xcbf46(%rip)        # 1893c8 <_DYNAMIC+0x248>
   bd482:	test   %rax,%rax
   bd485:	je     bd6fc <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   bd48b:	mov    %r12,%rsi
   bd48e:	jmp    bd50f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x20f>
   bd490:	mov    $0x8,%r13d
   bd496:	xor    %r12d,%r12d
   bd499:	test   %rbp,%rbp
   bd49c:	je     bd4aa <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1aa>
   bd49e:	mov    %r13,%rdi
   bd4a1:	mov    %r15,%rdx
   bd4a4:	call   *0xcbf36(%rip)        # 1893e0 <memcpy@GLIBC_2.14>
   bd4aa:	mov    %r12,(%r14)
   bd4ad:	mov    %r13,0x8(%r14)
   bd4b1:	mov    %rbp,0x10(%r14)
   bd4b5:	movzbl 0xc(%rsp),%eax
   bd4ba:	mov    %al,0x18(%r14)
   bd4be:	jmp    bd634 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   bd4c3:	dec    %edx
   bd4c5:	mov    %r15,%rdi
   bd4c8:	mov    %rdx,%r12
   bd4cb:	mov    %edx,%esi
   bd4cd:	call   127090 <num_bigint::bigint::shift::shr_round_down::<u32>>
   bd4d2:	mov    %eax,%ebp
   bd4d4:	cmpq   $0x0,0x20(%rbx)
   bd4d9:	je     bd547 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x247>
   bd4db:	mov    %r12,%rdx
   bd4de:	mov    %edx,%eax
   bd4e0:	and    $0x3f,%al
   bd4e2:	shr    $0x6,%edx
   bd4e5:	movq   $0xffffffffffffffff,0x10(%rsp)
   bd4ee:	mov    %r15,0x18(%rsp)
   bd4f3:	movzbl %al,%ecx
   bd4f6:	lea    0x38(%rsp),%rdi
   bd4fb:	lea    0x10(%rsp),%rsi
   bd500:	call   *0xcc10a(%rip)        # 189610 <_DYNAMIC+0x490>
   bd506:	jmp    bd554 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x254>
   bd508:	mov    $0x8,%eax
   bd50d:	xor    %esi,%esi
   bd50f:	test   %r12,%r12
   bd512:	je     bd53f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x23f>
   bd514:	mov    %r13,%rcx
   bd517:	mov    %rax,%r13
   bd51a:	mov    %rax,%rdi
   bd51d:	mov    %ebp,0xc(%rsp)
   bd521:	mov    %rsi,%rbp
   bd524:	mov    %rcx,%rsi
   bd527:	mov    %r15,%rdx
   bd52a:	call   *0xcbeb0(%rip)        # 1893e0 <memcpy@GLIBC_2.14>
   bd530:	mov    %r13,%rax
   bd533:	mov    %rbp,%rsi
   bd536:	mov    0xc(%rsp),%ebp
   bd53a:	jmp    bd5da <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bd53f:	xor    %r12d,%r12d
   bd542:	jmp    bd5da <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bd547:	lea    0x38(%rsp),%rdi
   bd54c:	mov    %r15,%rsi
   bd54f:	call   12b610 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   bd554:	test   %bpl,%bpl
   bd557:	je     bd56a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x26a>
   bd559:	lea    0x10(%rsp),%rdi
   bd55e:	lea    0x38(%rsp),%rsi
   bd563:	call   c1830 <<num_bigint::biguint::BigUint as core::ops::arith::Add<u32>>::add>
   bd568:	jmp    bd57e <<hyperreal::computable::node::EvaluationCache>::at_precision+0x27e>
   bd56a:	mov    0x48(%rsp),%rax
   bd56f:	mov    %rax,0x20(%rsp)
   bd574:	movups 0x38(%rsp),%xmm0
   bd579:	movaps %xmm0,0x10(%rsp)
   bd57e:	movzbl 0x28(%rbx),%eax
   bd582:	cmp    $0x1,%al
   bd584:	jne    bd599 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x299>
   bd586:	mov    0x10(%rsp),%rsi
   bd58b:	mov    $0x1,%bpl
   bd58e:	cmp    $0x4,%rsi
   bd592:	jae    bd5b3 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2b3>
   bd594:	xor    %r12d,%r12d
   bd597:	jmp    bd5d5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bd599:	mov    0x10(%rsp),%rsi
   bd59e:	mov    0x20(%rsp),%r12
   bd5a3:	test   %r12,%r12
   bd5a6:	movzbl %al,%eax
   bd5a9:	mov    $0x1,%ebp
   bd5ae:	cmovne %eax,%ebp
   bd5b1:	jmp    bd5d5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bd5b3:	mov    0x18(%rsp),%rdi
   bd5b8:	shl    $0x3,%rsi
   bd5bc:	mov    $0x8,%edx
   bd5c1:	call   *0xcbe41(%rip)        # 189408 <_DYNAMIC+0x288>
   bd5c7:	movq   $0x8,0x18(%rsp)
   bd5d0:	xor    %r12d,%r12d
   bd5d3:	xor    %esi,%esi
   bd5d5:	mov    0x18(%rsp),%rax
   bd5da:	mov    %rsi,0x10(%rsp)
   bd5df:	mov    %rax,0x18(%rsp)
   bd5e4:	mov    %r12,0x20(%rsp)
   bd5e9:	mov    %bpl,0x28(%rsp)
   bd5ee:	mov    0xfd41c(%rip),%ecx        # 1baa10 <hyperreal::computable::node::signed::ONE+0x20>
   bd5f4:	test   %ecx,%ecx
   bd5f6:	jne    bd6b6 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3b6>
   bd5fc:	lea    0xfd3ed(%rip),%rdx        # 1ba9f0 <hyperreal::computable::node::signed::ONE>
   bd603:	lea    0x38(%rsp),%rdi
   bd608:	lea    0x10(%rsp),%rsi
   bd60d:	call   c14e0 <<num_bigint::bigint::BigInt as core::ops::arith::Add<&num_bigint::bigint::BigInt>>::add>
   bd612:	lea    0x10(%rsp),%rdi
   bd617:	lea    0x38(%rsp),%rsi
   bd61c:	call   c2a00 <<num_bigint::bigint::BigInt as core::ops::bit::Shr<i32>>::shr>
   bd621:	movaps 0x10(%rsp),%xmm0
   bd626:	movaps 0x20(%rsp),%xmm1
   bd62b:	movups %xmm1,0x10(%r14)
   bd630:	movups %xmm0,(%r14)
   bd634:	mov    $0xffffffff,%esi
   bd639:	lock xadd %esi,(%rbx)
   bd63d:	dec    %esi
   bd63f:	mov    %esi,%eax
   bd641:	and    $0xbfffffff,%eax
   bd646:	neg    %eax
   bd648:	jno    bd690 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bd64a:	mov    %rbx,%rdi
   bd64d:	call   *0xcc0fd(%rip)        # 189750 <_DYNAMIC+0x5d0>
   bd653:	jmp    bd690 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bd655:	mov    %rbx,%rdi
   bd658:	mov    %rdx,%r15
   bd65b:	call   *0xcc0f7(%rip)        # 189758 <_DYNAMIC+0x5d8>
   bd661:	mov    %r15,%rdx
   bd664:	movzbl 0x8(%rbx),%eax
   bd668:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bd66d:	jne    bd342 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42>
   bd673:	movq   $0xffffffffffffffff,(%r14)
   bd67a:	mov    $0xffffffff,%esi
   bd67f:	lock xadd %esi,(%rbx)
   bd683:	dec    %esi
   bd685:	mov    %esi,%eax
   bd687:	and    $0xbfffffff,%eax
   bd68c:	neg    %eax
   bd68e:	jo     bd69f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x39f>
   bd690:	add    $0x68,%rsp
   bd694:	pop    %rbx
   bd695:	pop    %r12
   bd697:	pop    %r13
   bd699:	pop    %r14
   bd69b:	pop    %r15
   bd69d:	pop    %rbp
   bd69e:	ret
   bd69f:	mov    %rbx,%rdi
   bd6a2:	add    $0x68,%rsp
   bd6a6:	pop    %rbx
   bd6a7:	pop    %r12
   bd6a9:	pop    %r13
   bd6ab:	pop    %r14
   bd6ad:	pop    %r15
   bd6af:	pop    %rbp
   bd6b0:	jmp    *0xcc09a(%rip)        # 189750 <_DYNAMIC+0x5d0>
   bd6b6:	mov    %rax,%r12
   bd6b9:	mov    %rsi,%r15
   bd6bc:	lea    0xfd32d(%rip),%rax        # 1ba9f0 <hyperreal::computable::node::signed::ONE>
   bd6c3:	mov    %rax,0x60(%rsp)
   bd6c8:	lea    0x60(%rsp),%rax
   bd6cd:	mov    %rax,0x38(%rsp)
   bd6d2:	lea    0xfd337(%rip),%rdi        # 1baa10 <hyperreal::computable::node::signed::ONE+0x20>
   bd6d9:	lea    0xc76f8(%rip),%rcx        # 184dd8 <anon.9023bb4654a08944d5043612f590d0ff.3.llvm.2438011856832709387>
   bd6e0:	lea    0xc76d9(%rip),%r8        # 184dc0 <anon.9023bb4654a08944d5043612f590d0ff.2.llvm.2438011856832709387>
   bd6e7:	lea    0x38(%rsp),%rdx
   bd6ec:	mov    $0x1,%esi
   bd6f1:	call   *0xcbe71(%rip)        # 189568 <_DYNAMIC+0x3e8>
   bd6f7:	jmp    bd5fc <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2fc>
   bd6fc:	mov    $0x8,%edi
   bd701:	jmp    bd407 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x107>
   bd706:	mov    %rax,%r14
   bd709:	test   %r15,%r15
   bd70c:	je     bd72a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bd70e:	mov    %r15,%rsi
   bd711:	shl    $0x3,%rsi
   bd715:	mov    $0x8,%edx
   bd71a:	mov    %r12,%rdi
   bd71d:	call   *0xcbce5(%rip)        # 189408 <_DYNAMIC+0x288>
   bd723:	jmp    bd72a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bd725:	jmp    bd727 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x427>
   bd727:	mov    %rax,%r14
   bd72a:	mov    $0xffffffff,%esi
   bd72f:	lock xadd %esi,(%rbx)
   bd733:	dec    %esi
   bd735:	mov    %esi,%eax
   bd737:	and    $0xbfffffff,%eax
   bd73c:	neg    %eax
   bd73e:	jo     bd748 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x448>
   bd740:	mov    %r14,%rdi
   bd743:	call   183780 <_Unwind_Resume@plt>
   bd748:	mov    %rbx,%rdi
   bd74b:	call   *0xcbfff(%rip)        # 189750 <_DYNAMIC+0x5d0>
   bd751:	jmp    bd740 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x440>
   bd753:	call   *0xcbca7(%rip)        # 189400 <_DYNAMIC+0x280>

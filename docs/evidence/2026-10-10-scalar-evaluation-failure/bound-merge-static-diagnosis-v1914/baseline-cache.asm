
/home/tim/Documents/GitHub/workspace/csgrs/target/typed-api-prototype-v1773-domain-predicate-atan-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000bd170 <<hyperreal::computable::node::EvaluationCache>::at_precision>:
   bd170:	push   %rbp
   bd171:	push   %r15
   bd173:	push   %r14
   bd175:	push   %r13
   bd177:	push   %r12
   bd179:	push   %rbx
   bd17a:	sub    $0x68,%rsp
   bd17e:	mov    %rdi,%r14
   bd181:	mov    (%rsi),%rbx
   bd184:	test   %rbx,%rbx
   bd187:	je     bd1c7 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x57>
   bd189:	mov    (%rbx),%eax
   bd18b:	cmp    $0x3ffffffd,%eax
   bd190:	ja     bd4c5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bd196:	lea    0x1(%rax),%ecx
   bd199:	lock cmpxchg %ecx,(%rbx)
   bd19d:	jne    bd4c5 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x355>
   bd1a3:	movzbl 0x8(%rbx),%eax
   bd1a7:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bd1ac:	je     bd4e3 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x373>
   bd1b2:	lea    0x10(%rbx),%r15
   bd1b6:	sub    0x30(%rbx),%edx
   bd1b9:	jge    bd1d3 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x63>
   bd1bb:	movq   $0xffffffffffffffff,(%r14)
   bd1c2:	jmp    bd4a4 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   bd1c7:	movq   $0xffffffffffffffff,(%r14)
   bd1ce:	jmp    bd500 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bd1d3:	jne    bd246 <<hyperreal::computable::node::EvaluationCache>::at_precision+0xd6>
   bd1d5:	mov    0x20(%rbx),%rbp
   bd1d9:	lea    0x0(,%rbp,8),%r15
   bd1e1:	mov    %rbp,%rax
   bd1e4:	shr    $0x3d,%rax
   bd1e8:	setne  %al
   bd1eb:	movabs $0x7ffffffffffffff8,%rcx
   bd1f5:	cmp    %rcx,%r15
   bd1f8:	seta   %cl
   bd1fb:	or     %al,%cl
   bd1fd:	jne    bd275 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x105>
   bd1ff:	movzbl 0x28(%rbx),%eax
   bd203:	mov    %al,0xc(%rsp)
   bd207:	mov    0x18(%rbx),%rsi
   bd20b:	test   %r15,%r15
   bd20e:	je     bd300 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x190>
   bd214:	mov    %rsi,0x58(%rsp)
   bd219:	call   *0xcc011(%rip)        # 189230 <_DYNAMIC+0x240>
   bd21f:	mov    $0x8,%esi
   bd224:	mov    %r15,%rdi
   bd227:	call   *0xcc00b(%rip)        # 189238 <_DYNAMIC+0x248>
   bd22d:	test   %rax,%rax
   bd230:	je     bd56c <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   bd236:	mov    %rax,%r13
   bd239:	mov    %rbp,%r12
   bd23c:	mov    0x58(%rsp),%rsi
   bd241:	jmp    bd309 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x199>
   bd246:	cmp    $0x1,%edx
   bd249:	jne    bd282 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x112>
   bd24b:	mov    0x20(%rbx),%r12
   bd24f:	lea    0x0(,%r12,8),%r15
   bd257:	mov    %r12,%rax
   bd25a:	shr    $0x3d,%rax
   bd25e:	setne  %al
   bd261:	movabs $0x7ffffffffffffff8,%rcx
   bd26b:	cmp    %rcx,%r15
   bd26e:	seta   %cl
   bd271:	or     %al,%cl
   bd273:	je     bd2cd <<hyperreal::computable::node::EvaluationCache>::at_precision+0x15d>
   bd275:	xor    %edi,%edi
   bd277:	mov    %r15,%rsi
   bd27a:	call   *0xcc018(%rip)        # 189298 <_DYNAMIC+0x2a8>
   bd280:	ud2
   bd282:	mov    0x20(%rbx),%rax
   bd286:	test   %rax,%rax
   bd289:	je     bd2b0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x140>
   bd28b:	mov    0x18(%rbx),%rcx
   bd28f:	mov    %edx,%esi
   bd291:	mov    $0x7f,%edi
   bd296:	bsr    -0x8(%rcx,%rax,8),%rdi
   bd29c:	xor    $0x3f,%rdi
   bd2a0:	shl    $0x6,%rax
   bd2a4:	sub    %rdi,%rax
   bd2a7:	cmp    %rsi,%rax
   bd2aa:	jae    bd333 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1c3>
   bd2b0:	movups -0x89c87(%rip),%xmm0        # 33630 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102+0x10>
   bd2b7:	movaps %xmm0,0x20(%rsp)
   bd2bc:	movups -0x89ca3(%rip),%xmm0        # 33620 <anon.9023bb4654a08944d5043612f590d0ff.85.llvm.1768662447242434102>
   bd2c3:	movaps %xmm0,0x10(%rsp)
   bd2c8:	jmp    bd491 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x321>
   bd2cd:	movzbl 0x28(%rbx),%ebp
   bd2d1:	mov    0x18(%rbx),%r13
   bd2d5:	test   %r15,%r15
   bd2d8:	je     bd378 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x208>
   bd2de:	call   *0xcbf4c(%rip)        # 189230 <_DYNAMIC+0x240>
   bd2e4:	mov    $0x8,%esi
   bd2e9:	mov    %r15,%rdi
   bd2ec:	call   *0xcbf46(%rip)        # 189238 <_DYNAMIC+0x248>
   bd2f2:	test   %rax,%rax
   bd2f5:	je     bd56c <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3fc>
   bd2fb:	mov    %r12,%rsi
   bd2fe:	jmp    bd37f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x20f>
   bd300:	mov    $0x8,%r13d
   bd306:	xor    %r12d,%r12d
   bd309:	test   %rbp,%rbp
   bd30c:	je     bd31a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x1aa>
   bd30e:	mov    %r13,%rdi
   bd311:	mov    %r15,%rdx
   bd314:	call   *0xcbf36(%rip)        # 189250 <memcpy@GLIBC_2.14>
   bd31a:	mov    %r12,(%r14)
   bd31d:	mov    %r13,0x8(%r14)
   bd321:	mov    %rbp,0x10(%r14)
   bd325:	movzbl 0xc(%rsp),%eax
   bd32a:	mov    %al,0x18(%r14)
   bd32e:	jmp    bd4a4 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x334>
   bd333:	dec    %edx
   bd335:	mov    %r15,%rdi
   bd338:	mov    %rdx,%r12
   bd33b:	mov    %edx,%esi
   bd33d:	call   126f00 <num_bigint::bigint::shift::shr_round_down::<u32>>
   bd342:	mov    %eax,%ebp
   bd344:	cmpq   $0x0,0x20(%rbx)
   bd349:	je     bd3b7 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x247>
   bd34b:	mov    %r12,%rdx
   bd34e:	mov    %edx,%eax
   bd350:	and    $0x3f,%al
   bd352:	shr    $0x6,%edx
   bd355:	movq   $0xffffffffffffffff,0x10(%rsp)
   bd35e:	mov    %r15,0x18(%rsp)
   bd363:	movzbl %al,%ecx
   bd366:	lea    0x38(%rsp),%rdi
   bd36b:	lea    0x10(%rsp),%rsi
   bd370:	call   *0xcc10a(%rip)        # 189480 <_DYNAMIC+0x490>
   bd376:	jmp    bd3c4 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x254>
   bd378:	mov    $0x8,%eax
   bd37d:	xor    %esi,%esi
   bd37f:	test   %r12,%r12
   bd382:	je     bd3af <<hyperreal::computable::node::EvaluationCache>::at_precision+0x23f>
   bd384:	mov    %r13,%rcx
   bd387:	mov    %rax,%r13
   bd38a:	mov    %rax,%rdi
   bd38d:	mov    %ebp,0xc(%rsp)
   bd391:	mov    %rsi,%rbp
   bd394:	mov    %rcx,%rsi
   bd397:	mov    %r15,%rdx
   bd39a:	call   *0xcbeb0(%rip)        # 189250 <memcpy@GLIBC_2.14>
   bd3a0:	mov    %r13,%rax
   bd3a3:	mov    %rbp,%rsi
   bd3a6:	mov    0xc(%rsp),%ebp
   bd3aa:	jmp    bd44a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bd3af:	xor    %r12d,%r12d
   bd3b2:	jmp    bd44a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2da>
   bd3b7:	lea    0x38(%rsp),%rdi
   bd3bc:	mov    %r15,%rsi
   bd3bf:	call   12b480 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   bd3c4:	test   %bpl,%bpl
   bd3c7:	je     bd3da <<hyperreal::computable::node::EvaluationCache>::at_precision+0x26a>
   bd3c9:	lea    0x10(%rsp),%rdi
   bd3ce:	lea    0x38(%rsp),%rsi
   bd3d3:	call   c16a0 <<num_bigint::biguint::BigUint as core::ops::arith::Add<u32>>::add>
   bd3d8:	jmp    bd3ee <<hyperreal::computable::node::EvaluationCache>::at_precision+0x27e>
   bd3da:	mov    0x48(%rsp),%rax
   bd3df:	mov    %rax,0x20(%rsp)
   bd3e4:	movups 0x38(%rsp),%xmm0
   bd3e9:	movaps %xmm0,0x10(%rsp)
   bd3ee:	movzbl 0x28(%rbx),%eax
   bd3f2:	cmp    $0x1,%al
   bd3f4:	jne    bd409 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x299>
   bd3f6:	mov    0x10(%rsp),%rsi
   bd3fb:	mov    $0x1,%bpl
   bd3fe:	cmp    $0x4,%rsi
   bd402:	jae    bd423 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2b3>
   bd404:	xor    %r12d,%r12d
   bd407:	jmp    bd445 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bd409:	mov    0x10(%rsp),%rsi
   bd40e:	mov    0x20(%rsp),%r12
   bd413:	test   %r12,%r12
   bd416:	movzbl %al,%eax
   bd419:	mov    $0x1,%ebp
   bd41e:	cmovne %eax,%ebp
   bd421:	jmp    bd445 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2d5>
   bd423:	mov    0x18(%rsp),%rdi
   bd428:	shl    $0x3,%rsi
   bd42c:	mov    $0x8,%edx
   bd431:	call   *0xcbe41(%rip)        # 189278 <_DYNAMIC+0x288>
   bd437:	movq   $0x8,0x18(%rsp)
   bd440:	xor    %r12d,%r12d
   bd443:	xor    %esi,%esi
   bd445:	mov    0x18(%rsp),%rax
   bd44a:	mov    %rsi,0x10(%rsp)
   bd44f:	mov    %rax,0x18(%rsp)
   bd454:	mov    %r12,0x20(%rsp)
   bd459:	mov    %bpl,0x28(%rsp)
   bd45e:	mov    0xfd41c(%rip),%ecx        # 1ba880 <hyperreal::computable::node::signed::ONE+0x20>
   bd464:	test   %ecx,%ecx
   bd466:	jne    bd526 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x3b6>
   bd46c:	lea    0xfd3ed(%rip),%rdx        # 1ba860 <hyperreal::computable::node::signed::ONE>
   bd473:	lea    0x38(%rsp),%rdi
   bd478:	lea    0x10(%rsp),%rsi
   bd47d:	call   c1350 <<num_bigint::bigint::BigInt as core::ops::arith::Add<&num_bigint::bigint::BigInt>>::add>
   bd482:	lea    0x10(%rsp),%rdi
   bd487:	lea    0x38(%rsp),%rsi
   bd48c:	call   c2870 <<num_bigint::bigint::BigInt as core::ops::bit::Shr<i32>>::shr>
   bd491:	movaps 0x10(%rsp),%xmm0
   bd496:	movaps 0x20(%rsp),%xmm1
   bd49b:	movups %xmm1,0x10(%r14)
   bd4a0:	movups %xmm0,(%r14)
   bd4a4:	mov    $0xffffffff,%esi
   bd4a9:	lock xadd %esi,(%rbx)
   bd4ad:	dec    %esi
   bd4af:	mov    %esi,%eax
   bd4b1:	and    $0xbfffffff,%eax
   bd4b6:	neg    %eax
   bd4b8:	jno    bd500 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bd4ba:	mov    %rbx,%rdi
   bd4bd:	call   *0xcc0fd(%rip)        # 1895c0 <_DYNAMIC+0x5d0>
   bd4c3:	jmp    bd500 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x390>
   bd4c5:	mov    %rbx,%rdi
   bd4c8:	mov    %rdx,%r15
   bd4cb:	call   *0xcc0f7(%rip)        # 1895c8 <_DYNAMIC+0x5d8>
   bd4d1:	mov    %r15,%rdx
   bd4d4:	movzbl 0x8(%rbx),%eax
   bd4d8:	cmpq   $0xffffffffffffffff,0x10(%rbx)
   bd4dd:	jne    bd1b2 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42>
   bd4e3:	movq   $0xffffffffffffffff,(%r14)
   bd4ea:	mov    $0xffffffff,%esi
   bd4ef:	lock xadd %esi,(%rbx)
   bd4f3:	dec    %esi
   bd4f5:	mov    %esi,%eax
   bd4f7:	and    $0xbfffffff,%eax
   bd4fc:	neg    %eax
   bd4fe:	jo     bd50f <<hyperreal::computable::node::EvaluationCache>::at_precision+0x39f>
   bd500:	add    $0x68,%rsp
   bd504:	pop    %rbx
   bd505:	pop    %r12
   bd507:	pop    %r13
   bd509:	pop    %r14
   bd50b:	pop    %r15
   bd50d:	pop    %rbp
   bd50e:	ret
   bd50f:	mov    %rbx,%rdi
   bd512:	add    $0x68,%rsp
   bd516:	pop    %rbx
   bd517:	pop    %r12
   bd519:	pop    %r13
   bd51b:	pop    %r14
   bd51d:	pop    %r15
   bd51f:	pop    %rbp
   bd520:	jmp    *0xcc09a(%rip)        # 1895c0 <_DYNAMIC+0x5d0>
   bd526:	mov    %rax,%r12
   bd529:	mov    %rsi,%r15
   bd52c:	lea    0xfd32d(%rip),%rax        # 1ba860 <hyperreal::computable::node::signed::ONE>
   bd533:	mov    %rax,0x60(%rsp)
   bd538:	lea    0x60(%rsp),%rax
   bd53d:	mov    %rax,0x38(%rsp)
   bd542:	lea    0xfd337(%rip),%rdi        # 1ba880 <hyperreal::computable::node::signed::ONE+0x20>
   bd549:	lea    0xc76f8(%rip),%rcx        # 184c48 <anon.9023bb4654a08944d5043612f590d0ff.3.llvm.1768662447242434102>
   bd550:	lea    0xc76d9(%rip),%r8        # 184c30 <anon.9023bb4654a08944d5043612f590d0ff.2.llvm.1768662447242434102>
   bd557:	lea    0x38(%rsp),%rdx
   bd55c:	mov    $0x1,%esi
   bd561:	call   *0xcbe71(%rip)        # 1893d8 <_DYNAMIC+0x3e8>
   bd567:	jmp    bd46c <<hyperreal::computable::node::EvaluationCache>::at_precision+0x2fc>
   bd56c:	mov    $0x8,%edi
   bd571:	jmp    bd277 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x107>
   bd576:	mov    %rax,%r14
   bd579:	test   %r15,%r15
   bd57c:	je     bd59a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bd57e:	mov    %r15,%rsi
   bd581:	shl    $0x3,%rsi
   bd585:	mov    $0x8,%edx
   bd58a:	mov    %r12,%rdi
   bd58d:	call   *0xcbce5(%rip)        # 189278 <_DYNAMIC+0x288>
   bd593:	jmp    bd59a <<hyperreal::computable::node::EvaluationCache>::at_precision+0x42a>
   bd595:	jmp    bd597 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x427>
   bd597:	mov    %rax,%r14
   bd59a:	mov    $0xffffffff,%esi
   bd59f:	lock xadd %esi,(%rbx)
   bd5a3:	dec    %esi
   bd5a5:	mov    %esi,%eax
   bd5a7:	and    $0xbfffffff,%eax
   bd5ac:	neg    %eax
   bd5ae:	jo     bd5b8 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x448>
   bd5b0:	mov    %r14,%rdi
   bd5b3:	call   1835f0 <_Unwind_Resume@plt>
   bd5b8:	mov    %rbx,%rdi
   bd5bb:	call   *0xcbfff(%rip)        # 1895c0 <_DYNAMIC+0x5d0>
   bd5c1:	jmp    bd5b0 <<hyperreal::computable::node::EvaluationCache>::at_precision+0x440>
   bd5c3:	call   *0xcbca7(%rip)        # 189270 <_DYNAMIC+0x280>

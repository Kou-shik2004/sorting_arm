# Lessons

Patterns that would have prevented a correction. One entry per pattern, not per incident.

## Container lifecycle is driven from VS Code, not the host shell

Koushik starts and stops the dev container through the Dev Containers extension. Host-side `docker compose` commands are for inspection only — `docker exec`, `docker compose config`, `docker ps`.

When a change to `docker-compose.yaml` or `.devcontainer/devcontainer.json` needs to take effect, give the VS Code command palette action, not the compose command. `shutdownAction: "none"` means closing the window leaves the container running, so reopening reattaches to the stale container and the change appears to have done nothing.

Read `.devcontainer/devcontainer.json` before advising on a restart. The answer is in that file.

## Establish what kind of understanding is needed before naming a single resource

The study guide took three positions in three turns: documentation only, then four chapters of Craig, then "28 pages of Craig". Each move came from Koushik pushing back, not from analysis. Three answers, no reasoning, and the flipping destroyed confidence in all three — correctly, because none of them was derived.

**Root cause: the requirement was never established.** Koushik configures MoveIt, TF2 and OMPL and implements none of them — D11 rules that out by decision. So what he needs is to *explain* a singularity, never to compute rank; to know what IK failing means, never to solve IK. Conceptual, not computational. That one question, asked in the first turn, fixes the answer permanently — and the answer is not Craig, because Craig is a derivation textbook and reading it for concepts alone throws away most of what it is for. For explain-it understanding, video lectures and prose beat a derivation textbook; Modern Robotics' free lectures were the right answer the whole time.

**Ask first, for any resource recommendation:** what capability is being bought, and is it "explain it" or "compute it"? Then pick the medium that fits. Naming resources before answering that is guessing.

**Second failure, and the one that reads as hallucination: never state a page count, section number or reading duration for a source not open in front of me.** The guide carried "~20 pages", "~8 pages", "about two hours" for a book never seen. Invented precision looks like authority and is the fastest possible way to lose a reader's trust. Name the concept, and map it to sections only once the file is actually available.

**Third: when a recommendation is challenged, re-derive it from the requirements — do not move toward the objection.** Accommodation produces a new position every turn. Re-derivation produces the same position with a better explanation, or a genuinely different one with a reason attached. Only the second is worth anything.

## A missing local resource is a request to Koushik, not a web hunt

Reference repos live in `context/` (or arrive when Koushik clones them). When a note points at a path that is not mounted, the move is one sentence: "the repo isn't here — clone it into context/ and I'll read it." Searching the web for a repo or guessing at someone's GitHub username invents facts about a person and wastes turns; the authoritative copy is always the one Koushik provides.

## Session logs record conversations, not Koushik's calendar

Never narrate or judge how Koushik spent his time from session summaries — they log what Claude did in a window, not his day (interviews, environment setup, reading happen off-log). "You spent two days on X" derived from observation timestamps is invented, and pace judgments are off-limits unless he asks for one. Compare projects on the axis he names; completion status is one fact, not the verdict.

## A comparison lists what actually differs, and nothing else

Comparing sorting_arm to a reference repo produced six manufactured differences: a star rating for "parameterized bins" (a line lifted from our own PROJECT.md that does not survive Gazebo — bins are world content), a perception gap that was really his shipped choice rated against our plan's current scope, and "sorting is more advanced than pick and place" (it is pick and place with a lookup before the place pose — same MoveIt calls, same grasp, same attach/detach).

**Strip the comparison to what differs, then stop.** Same technique on both sides is not a difference. A capability we have not chosen yet is not a gap — it is an open choice. A claim taken from our own docs still needs checking against reality before it becomes a point in our favour. And never compare a shipped repo against an unbuilt plan on the completion axis unless Koushik asks — the ask is plan versus plan, both as finished.

Padding a comparison to make it look thorough destroys the credibility of the one or two differences that are real.

## A previous session's conclusion is input, not authority

The Modern Robotics recommendation was inherited from the prior session and a plan was built on top of it without re-derivation; Koushik rejected the plan for exactly that. Before building on any earlier conclusion — including one's own — walk it back to the project's decisions and timeline and re-derive it. If it re-derives, say so with the chain shown; if it does not, the inherited position was a guess wearing a memory.

Two rules the re-derivation produced. **Match the resource unit to the requirement unit:** a two-sentence capability never justifies a chapter — but a chapter's *section*, named with pages counted from an open copy, is a perfectly good unit. "Books are too much" is exactly as underived as "read the book". **A number may only stand next to a source that was actually open:** the distrusted "28 pages of Craig" and the accepted "29 pages of MR" differ not in size but in that every one of the 29 is named, was read, and can be checked.

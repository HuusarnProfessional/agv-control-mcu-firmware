# implementation rules

These are general rules for code under `core/control/`.

Task-specific behavior belongs in the relevant `logic.md` or design document, not here.

## style

- Use Allman style braces.
- Prefer explicit `if` statements over `?:`.
- Prioritize readability over compactness.
- Keep statements on one line.
- Do not break lines inside expressions, conditions, assignments, or function calls.
- If a line becomes long, keep it long.
- Prefer horizontal scrolling over line breaks.
- Use descriptive names.
- Avoid unclear abbreviations unless they are already standard in the codebase.
- Use fixed-width integer types such as `std::uint32_t` and `std::int32_t`.
- Keep whitespace clean and consistent.
- Avoid clever one-liners.
- no capital letters

## control flow

- Prefer simple control flow.
- Prefer early return over deep nesting.
- Keep conditions easy to read.
- If a condition becomes hard to read, split it into named boolean variables, not line breaks.
- One clear step per block.
- Do not hide decisions inside compact expressions.
- Do not assign boolean state directly from `&&` or `||` expressions when `if` statements are clearer.
- Avoid double checks when the contract already guarantees something.
- Do not stack defensive checks that only add noise.
- In stub code, keep parsing result and final function result as separate steps.
- Do not combine parsing calls with placeholder returns in the same expression.

## functions

- Keep functions focused on one responsibility.
- Keep functions short when possible.
- If a function starts doing multiple jobs, split it.
- Use helper functions when they improve readability.
- Use helper functions to keep the main part readable.
- The main function should show the main idea clearly.
- Detailed steps can live in helper functions for anyone who wants to read deeper.
- Do not move trivial code into helpers if that makes the flow harder to follow.

## file responsibilities

- `pipeline` files coordinate and connect modules.
- `logic` files contain the actual decision logic.
- `tuning` files hold constants and configuration.
- `internal/` files prepare data or handle internal sub-logic.
- Do not mix middleware parsing, board behavior, and core decision logic in the same file.

## readability

- Prefer explicit intermediate variables over hard-to-read expressions.
- Keep math readable.
- If a formula is important, write it in a step-by-step way.
- Do not use line breaks as a readability tool inside code statements.
- Add short comments only where they help understanding.
- Do not add comments that repeat the code.
- In handlers and parsers, prefer named intermediate variables like `parsed_ok` when they make the flow clearer.

## implementation discipline

- Do not add debug-only complexity unless it is needed now.
- Do not add extra outputs "for later" unless they are already required.
- Do not over-generalize for hardware we do not have yet.
- Keep extension points clean, but keep current implementation concrete.
- Put magic numbers in tuning/config, not inline in logic.
- When in doubt, choose the version that is easier to read in six months.

# Debug config params

## Compiler options

Set to compiler ```version 5```

## Simulator config

**CPU DLL:**

```text
SARMCM3.DLL
```

**Parameter:**

```text
-REMAP
```

**Dialog DLL:**

```text
DARMSTM.DLL
```

**Parameter:**

```text
-pSTM32F103RB
```

## ST-link Debugger

Debugger name: ```ST-link Debugger```

**Driver DLL:**

```text
SARMCM3.DLL
```

**Parameter:**

None

**Dialog DLL:**

```text
TARMSTL.DLL
```

**Parameter:**

```text
-pSTM32F103RB
```

### Additionnal settings

ST-link Debugger -> Settings -> Flash and Download -> Check ```Reset and Run```

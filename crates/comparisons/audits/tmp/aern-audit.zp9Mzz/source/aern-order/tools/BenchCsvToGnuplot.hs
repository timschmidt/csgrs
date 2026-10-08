-- currently bothballed, will probably be never needed
module Main where

import System.Environment
import System.Exit
import System.Directory
import qualified Data.Map as Map
import qualified Data.Set as Set
import qualified Data.List as List 

main =
    do
    args <- getArgs
    inFileNames <- checkArgs args
    recordLists <- mapM readCSV inFileNames
    let benchRecords = concat $ map snd recordLists
    let outputData = processData $ map separateImprecision benchRecords
    let outFileNames = map makeOutFileName $ map fst outputData 
    mapM_ writeOutputFile $ zip outFileNames outputData
    writeFile "plotAll.gnuplot" $ makeGnuplot inFileNames outFileNames

checkArgs [] = usageAndQuit
checkArgs inFileNames =
    do
    results <- mapM doesFileExist inFileNames
    case and results of
        True -> return inFileNames
        False -> usageAndQuit
        
usageAndQuit =
     do
     progName <- getProgName
     putStrLn $ "usage: " ++ progName ++ " <file1>.csv [<file2>.csv ...]"
     exitFailure

slash2dash '/' = '-'
slash2dash c = c

writeOutputFile (outFileName, (name, benchRecords)) =
    do
    putStrLn outFileName
    writeCSV outFileName (["Imprecision", "Mean"], benchRecords)
    
makeOutFileName name =
    (map space2underscore name) ++ ".csv"
    
space2underscore ' ' = '_'
space2underscore c = c

separateImprecision record =    
    Map.insert "Name" name $
    Map.insert "Imprecision" imprecision $
    record
    where
    name = map slash2dash nameSlashes
    imprecision = tail slashImprecision
    (nameSlashes, slashImprecision) = splitAt slashIndex nameSlashImprecision
    slashIndex = last $ List.elemIndices '/' nameSlashImprecision
    nameSlashImprecision = lookupKey record "Name" 
    
processData benchRecords =
    map extractName names
    where
    names = Set.toList $ Set.fromList $ map (\r -> lookupKey r "Name") benchRecords
    extractName name =
        (name, 
         filter (\r -> lookupKey r "Name" == name) benchRecords)
    
makeGnuplot inFileNames outFileNames =
    unlines $
        [
         "# AUTO GENERATED FROM " ++ (List.intercalate ", " inFileNames)
         ,"set term postscript eps enhanced"
         ,"set output \"plotAll.eps\""
         ,"set title 'computation time based on result imprecision'"
         ,"set key right bottom"
         ,"set key box linestyle 1"
         ,"set key spacing 1.5"
         ,"set logscale x"
         ,"set logscale y"
         ,"set xlabel 'imprecision'"
         ,"set ylabel 'time'"
         ,"set datafile separator \",\""
        ]
        ++
        [makePlotLine outFileNames]
    where
    makePlotLine outFileNames =
        "plot" ++ (List.intercalate "," (map makePlotImport outFileNames))
    makePlotImport outFileName = 
        " \"" ++ outFileName ++ "\" using 1:2"
    
type Record = Map.Map String String
type Sheet = ([String], [Record])

lookupKey :: Record -> String -> String
lookupKey record key =
    case Map.lookup key record of
        Nothing -> "" -- error $ "key " ++ show key ++ " not found in record " ++ show record
        Just value -> value


writeCSV :: FilePath -> Sheet -> IO ()
writeCSV filePath (keys, records) =
    do
    writeFile filePath $ unlines $ map formatLine $ (keys : dataLists)
    where
    dataLists =
        map makeDataItems records
    makeDataItems record =
        [lookupKey record key | key <- keys ]
    formatLine items =
        separateWith "," $ map show items -- must not contain a comma

separateWith sep [] = ""
separateWith sep [a] = a
separateWith sep (h:t) = h ++ sep ++ (separateWith sep t)


readCSV :: FilePath -> IO Sheet
readCSV filePath =
    do
    contents <- readFile filePath
    return $ processContents contents
    where
    processContents contents =
        (header, recordMaps)
        where
        hdrRecords@(header : records) = parseCSV contents
        recordMaps = 
            map snd $
               indexRecordsByKeysAndHeader [] hdrRecords   

indexRecordsByKeysAndHeader keys (header : records) =
    map getKeysAndMap records
    where
    getKeysAndMap record =
        (getKeys record, getMap record)
    getKeys record =
        map getKey keyIndices
        where
        getKey keyIx = record !! keyIx
    keyIndices =
        map keyIndex keys
    keyIndex key =
        case List.elemIndex key header of
            Nothing -> error $ "key " ++ show key ++ " not found in the header " ++ show header
            Just ix -> ix
    getMap record =
        Map.fromList $ zip header record

-- | parse records and their fields from the contents of a CSV file
parseCSV :: String -> [[String]]
parseCSV contents =
    records
    where
    records =
        map parseLine $ lines contents
    parseLine line =
        state1 0 [] "" line
        where
        -- expecting new field or end of line; initial state
        state1 pos revPrevItems revPrevOutput [] =
            reverse $ reverse revPrevOutput : revPrevItems
        state1 pos revPrevItems revPrevOutput "\x0D" = -- DOS end of line
            reverse $ reverse revPrevOutput : revPrevItems
        state1 pos revPrevItems revPrevOutput (',' : cs) =
            state1 (pos + 1) (reverse revPrevOutput : revPrevItems) "" cs
        state1 pos revPrevItems revPrevOutput ('"' : cs) =
            state3 (pos + 1) revPrevItems revPrevOutput cs
        state1 pos revPrevItems revPrevOutput (c : cs) =
            state2 (pos + 1) revPrevItems (c : revPrevOutput) cs

        -- reading a field with no double quotes
        state2 pos revPrevItems revPrevOutput [] =
            reverse $ reverse revPrevOutput : revPrevItems
        state2 pos revPrevItems revPrevOutput "\x0D" = -- DOS end of line
            reverse $ reverse revPrevOutput : revPrevItems
        state2 pos revPrevItems revPrevOutput (',' : cs) =
            state1 (pos + 1) (reverse revPrevOutput : revPrevItems) "" cs
        state2 pos revPrevItems revPrevOutput (c : cs) =
            state2 (pos + 1) revPrevItems (c : revPrevOutput) cs

        -- reading a field in double quotes
        state3 pos revPrevItems revPrevOutput [] =
            parseerror pos
        state3 pos revPrevItems revPrevOutput ('"' : cs) =
            state4 (pos + 1) revPrevItems revPrevOutput cs
        state3 pos revPrevItems revPrevOutput (c : cs) =
            state3 (pos + 1) revPrevItems (c : revPrevOutput) cs

        -- reading a field in double quotes and just found a double quote
        -- that could be the closing one or an inner one
        state4 pos revPrevItems revPrevOutput [] =
            reverse $ reverse revPrevOutput : revPrevItems
        state4 pos revPrevItems revPrevOutput "\x0D" = -- DOS end of line
            reverse $ reverse revPrevOutput : revPrevItems
        state4 pos revPrevItems revPrevOutput (',' : cs) =
            state1 (pos + 1) (reverse revPrevOutput : revPrevItems) "" cs
        state4 pos revPrevItems revPrevOutput (c : cs) =
            state3 (pos + 1) revPrevItems (c : revPrevOutput) cs

        parseerror pos =
            error $
                "parse error in CVS file at pos:\n"
                ++ take pos line ++ "\n"
                ++ replicate pos ' ' ++ drop pos line
